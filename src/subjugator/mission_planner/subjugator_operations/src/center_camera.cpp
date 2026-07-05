#include "center_camera.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

CenterCamera::CenterCamera(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList CenterCamera::providedPorts()
{
    return { BT::InputPort<std::string>("label", "table", "Target class to center on"),
             BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
             BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
             BT::InputPort<double>("tol_norm", 0.05, "Centered when |ex|,|ey| < tol (fraction of half-image)"),
             BT::InputPort<double>("kp", 0.5, "Proportional gain: normalized error -> meters/step"),
             BT::InputPort<double>("max_step", 0.25, "Max XY correction per step (m)"),
             BT::InputPort<double>("settle_pos_tol", 0.10, "Re-evaluate once within this distance of last step (m)"),
             BT::InputPort<int>("settle_ticks", 3,
                                "Consecutive fresh centered frames required for SUCCESS (a fresh frame that is "
                                "off-center OR missing the target resets the count)"),
             // Losing should be harder than winning: 3 consecutive fresh frames
             // lock a target (SelectTarget), 5 declare it lost. 1 = legacy
             // fail-on-first-miss. Counted per distinct detection frame (stamp),
             // NOT per BT tick, so the tolerance window is rate-independent
             // (~0.5 s at a 10 Hz pool detection stream). Caveat: an unstamped
             // publisher (stamp==0) degrades the count to per-tick — see
             // detection_gate.hpp.
             BT::InputPort<int>("miss_frames", 5,
                                "Consecutive fresh detection frames without the target before FAILURE"),
             // Pixel->body mapping. Defaults are placeholders; tune empirically
             // in sim (the down-cam is pitched 90deg + an optical-frame rotation,
             // so the surge/sway assignment and signs are not obvious).
             BT::InputPort<bool>("swap_axes", false, "If true, image-x drives surge and image-y drives sway"),
             BT::InputPort<double>("map_x_sign", 1.0, "Sign on the surge (body x) correction"),
             BT::InputPort<double>("map_y_sign", 1.0, "Sign on the sway (body y) correction"),
             BT::InputPort<std::shared_ptr<Context>>("ctx") };
}

BT::NodeStatus CenterCamera::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "CenterCamera: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    settling_ = false;
    step_dist_ = 0.0;
    in_tol_count_ = 0;
    // Seed the gate with whatever frame is already cached so only frames
    // captured AFTER this activation count. Matters most for the second hone
    // in ApproachAndGrasp: it starts right after the lift RelativeMove, and
    // acting on a cached pre-lift frame is the overshoot bug the freshness
    // gate exists to prevent — across node restarts, not just within one.
    std::string camera = "down";
    (void)getInput("camera", camera);
    gate_.seed_from(ctx_->detections_for(camera));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CenterCamera::onRunning()
{
    // Inputs
    std::string label = "table";
    std::string camera = "down";
    double min_conf = 0.30;
    double tol_norm = 0.05;
    double kp = 0.5;
    double max_step = 0.25;
    double settle_pos_tol = 0.10;
    int settle_ticks = 3;
    int miss_frames = 5;
    bool swap_axes = false;
    double map_x_sign = 1.0;
    double map_y_sign = 1.0;

    (void)getInput("label", label);
    (void)getInput("camera", camera);
    (void)getInput("min_conf", min_conf);
    (void)getInput("tol_norm", tol_norm);
    (void)getInput("kp", kp);
    (void)getInput("max_step", max_step);
    (void)getInput("settle_pos_tol", settle_pos_tol);
    (void)getInput("settle_ticks", settle_ticks);
    (void)getInput("miss_frames", miss_frames);
    (void)getInput("swap_axes", swap_axes);
    (void)getInput("map_x_sign", map_x_sign);
    (void)getInput("map_y_sign", map_y_sign);

    // Current pose (needed for the settle check and to compose world goals).
    std::optional<geometry_msgs::msg::Pose> current;
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
        {
            current = ctx_->latest_odom->pose.pose;
        }
    }
    if (!current)
    {
        return BT::NodeStatus::RUNNING;  // wait for odom
    }

    // If a correction is in flight, wait until the commanded step has been
    // *traversed* before re-evaluating. Measured as distance moved from where we
    // issued the step (private state), so it works for small steps too -- unlike
    // proximity-to-goal, which is trivially satisfied when step < settle_pos_tol.
    if (settling_)
    {
        double moved = std::hypot(current->position.x - step_start_x_, current->position.y - step_start_y_);
        if (moved < step_dist_ - settle_pos_tol)
        {
            return BT::NodeStatus::RUNNING;  // still traversing the step
        }
        settling_ = false;
    }

    // Image size for normalization. Cold start (no frame yet) -> keep waiting.
    uint32_t W = 0, H = 0;
    if (!ctx_->image_size_for(camera, W, H))
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "CenterCamera: waiting for %s image size",
                             camera.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Best detection of the requested label. No detection message at all yet
    // (cold start, e.g. YOLO still warming up) -> keep waiting like the image
    // size above; the wrapping <Timeout> bounds the wait.
    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
    if (!arr)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "CenterCamera: waiting for %s detections",
                             camera.c_str());
        return BT::NodeStatus::RUNNING;
    }
    yolo_msgs::msg::Detection const* best = nullptr;
    double best_conf = 0.0;
    for (auto const& d : arr->detections)
    {
        if (d.class_name == label && d.score >= min_conf && d.score > best_conf)
        {
            best = &d;
            best_conf = d.score;
        }
    }

    // Frame gate (see detection_gate.hpp): only ever act on a detection
    // frame newer than the one we last considered — never re-correct from a
    // pre-move observation (would overshoot), and settle_ticks counts distinct
    // frames rather than fast re-reads of one stale frame. A fresh frame
    // WITHOUT the target is a miss; only miss_frames consecutive fresh misses
    // mean the target is genuinely lost (-> FAILURE, so a Fallback can degrade
    // to dead-reckon). A single flickered frame no longer aborts centering.
    // Known tradeoff: if the publisher dies mid-centering the last array stays
    // stale forever (kStale -> RUNNING); the wrapping <Timeout> is the bound
    // for that case, same as when the target-present frame goes stale.
    std::int64_t stamp_ns = rclcpp::Time(arr->header.stamp).nanoseconds();
    switch (gate_.update(best != nullptr, stamp_ns, miss_frames))
    {
        case detection_gate::MissGate::Verdict::kStale:
            return BT::NodeStatus::RUNNING;  // same frame as last tick -> hold
        case detection_gate::MissGate::Verdict::kMiss:
            // Tolerated flicker — but it still breaks the "consecutive fresh
            // centered frames" chain, so the settle confirmation starts over.
            in_tol_count_ = 0;
            return BT::NodeStatus::RUNNING;
        case detection_gate::MissGate::Verdict::kLost:
            RCLCPP_WARN(ctx_->logger(), "CenterCamera: '%s' lost (%d consecutive frames without it)", label.c_str(),
                        gate_.misses);
            return BT::NodeStatus::FAILURE;
        case detection_gate::MissGate::Verdict::kHit:
            break;  // fresh frame with the target -> evaluate below
    }

    // Normalized centroid error: 0 = centered, +/-1 = image edge.
    double ex = (best->bbox.center.position.x - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0);
    double ey = (best->bbox.center.position.y - static_cast<double>(H) / 2.0) / (static_cast<double>(H) / 2.0);

    if (std::abs(ex) < tol_norm && std::abs(ey) < tol_norm)
    {
        if (++in_tol_count_ >= settle_ticks)
        {
            RCLCPP_INFO(ctx_->logger(), "CenterCamera: centered (ex=%.3f ey=%.3f)", ex, ey);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;  // hold and confirm over fresh frames
    }
    in_tol_count_ = 0;

    // Map image error -> body-frame step (surge=x, sway=y). swap_axes/signs are
    // tuned in sim against the real down-cam mounting.
    double surge_err = swap_axes ? ey : ex;
    double sway_err = swap_axes ? ex : ey;
    double step_surge = std::clamp(map_x_sign * kp * surge_err, -max_step, max_step);
    double step_sway = std::clamp(map_y_sign * kp * sway_err, -max_step, max_step);

    // Rotate body-frame step into world XY by the current yaw; hold z and yaw.
    auto const& q = current->orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double world_dx = step_surge * std::cos(yaw) - step_sway * std::sin(yaw);
    double world_dy = step_surge * std::sin(yaw) + step_sway * std::cos(yaw);

    geometry_msgs::msg::Pose goal = *current;
    goal.position.x += world_dx;
    goal.position.y += world_dy;
    // z and orientation held.

    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }

    // Record private settle state: the pose we issued the step from and the
    // step's horizontal length. The next correction waits until this step is
    // traversed (above) and a newer frame arrives (frame gate above).
    step_start_x_ = current->position.x;
    step_start_y_ = current->position.y;
    step_dist_ = std::hypot(world_dx, world_dy);
    settling_ = true;

    RCLCPP_INFO(ctx_->logger(), "CenterCamera: ex=%.3f ey=%.3f -> surge=%.3f sway=%.3f", ex, ey, step_surge, step_sway);
    return BT::NodeStatus::RUNNING;
}

void CenterCamera::onHalted()
{
}
