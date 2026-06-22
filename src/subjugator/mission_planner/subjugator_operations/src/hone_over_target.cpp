#include "hone_over_target.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

HoneOverTarget::HoneOverTarget(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList HoneOverTarget::providedPorts()
{
    return { BT::InputPort<std::string>("label", "table", "Target class to center on"),
             BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
             BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
             BT::InputPort<double>("tol_norm", 0.05, "Centered when |ex|,|ey| < tol (fraction of half-image)"),
             BT::InputPort<double>("kp", 0.5, "Proportional gain: normalized error -> meters/step"),
             BT::InputPort<double>("max_step", 0.25, "Max XY correction per step (m)"),
             BT::InputPort<double>("settle_pos_tol", 0.10, "Re-evaluate once within this distance of last step (m)"),
             BT::InputPort<int>("settle_ticks", 3, "Consecutive centered reads required for SUCCESS"),
             // Pixel->body mapping. Defaults are placeholders; tune empirically
             // in sim (the down-cam is pitched 90deg + an optical-frame rotation,
             // so the surge/sway assignment and signs are not obvious).
             BT::InputPort<bool>("swap_axes", false, "If true, image-x drives surge and image-y drives sway"),
             BT::InputPort<double>("map_x_sign", 1.0, "Sign on the surge (body x) correction"),
             BT::InputPort<double>("map_y_sign", 1.0, "Sign on the sway (body y) correction"),
             BT::InputPort<std::shared_ptr<Context>>("ctx") };
}

BT::NodeStatus HoneOverTarget::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "HoneOverTarget: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    settling_ = false;
    step_dist_ = 0.0;
    last_acted_ns_ = -1;
    in_tol_count_ = 0;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoneOverTarget::onRunning()
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
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000,
                             "HoneOverTarget: waiting for %s image size", camera.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Best detection of the requested label.
    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
    yolo_msgs::msg::Detection const* best = nullptr;
    double best_conf = 0.0;
    if (arr)
    {
        for (auto const& d : arr->detections)
        {
            if (d.class_name == label && d.score >= min_conf && d.score > best_conf)
            {
                best = &d;
                best_conf = d.score;
            }
        }
    }
    if (!best)
    {
        // Target not visible -> let a Fallback degrade to dead-reckon.
        return BT::NodeStatus::FAILURE;
    }

    // Only ever act on a detection frame newer than the one we last used, so we
    // never re-correct from a pre-move observation (would overshoot) and so the
    // settle_ticks confirmation counts distinct frames rather than fast re-reads
    // of one stale frame. stamp==0 (publisher not stamping) -> treat as fresh so
    // we never stall; pacing then falls back to the step-traversal gate above.
    std::int64_t stamp_ns = rclcpp::Time(arr->header.stamp).nanoseconds();
    if (stamp_ns != 0 && stamp_ns <= last_acted_ns_)
    {
        return BT::NodeStatus::RUNNING;  // hold until a fresh frame arrives
    }

    // Normalized centroid error: 0 = centered, +/-1 = image edge.
    double ex = (best->bbox.center.position.x - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0);
    double ey = (best->bbox.center.position.y - static_cast<double>(H) / 2.0) / (static_cast<double>(H) / 2.0);

    if (std::abs(ex) < tol_norm && std::abs(ey) < tol_norm)
    {
        last_acted_ns_ = stamp_ns;
        if (++in_tol_count_ >= settle_ticks)
        {
            RCLCPP_INFO(ctx_->logger(), "HoneOverTarget: centered (ex=%.3f ey=%.3f)", ex, ey);
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

    // Record private settle state: the pose we issued the step from, the step's
    // horizontal length, and the frame we acted on. The next correction waits
    // until this step is traversed (above) and a newer frame arrives.
    step_start_x_ = current->position.x;
    step_start_y_ = current->position.y;
    step_dist_ = std::hypot(world_dx, world_dy);
    last_acted_ns_ = stamp_ns;
    settling_ = true;

    RCLCPP_INFO(ctx_->logger(), "HoneOverTarget: ex=%.3f ey=%.3f -> surge=%.3f sway=%.3f", ex, ey, step_surge,
                step_sway);
    return BT::NodeStatus::RUNNING;
}

void HoneOverTarget::onHalted()
{
}
