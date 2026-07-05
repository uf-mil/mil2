#include "descend_until_detected.hpp"

#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>

DescendUntilDetected::DescendUntilDetected(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList DescendUntilDetected::providedPorts()
{
    return {
        BT::InputPort<std::string>("label", "table", "YOLO class label to wait for"),
        BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
        BT::InputPort<double>("min_conf", 0.40, "Minimum detection confidence"),
        // "3 to win", mirroring SelectTarget's lock: a single misclassified
        // frame (pool-floor glare scored as 'table') must not stop the descent
        // ~2 m high. Frames are counted by stamp (fresh frames only), so at
        // pool detection rates confirmation costs ~0.1-0.3 s. Stepping PAUSES
        // while a hit streak is live (and until the first post-start frame has
        // been evaluated), so a target visible at the start still confirms
        // with zero extra steps; only fully-missed frames advance the descent.
        BT::InputPort<int>("confirm_frames", 3,
                           "Consecutive fresh detection frames with the label required to stop descending"),
        BT::InputPort<double>("step_m", 0.20, "Downward step per cycle (m)"),
        BT::InputPort<double>("pos_tol", 0.10, "Goal-reached tolerance (m)"),
        BT::InputPort<int>("max_steps", 12, "Max descend steps before FAILURE"),
        BT::InputPort<int>("timeout_msec", 45000, "Overall timeout (ms)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus DescendUntilDetected::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "DescendUntilDetected: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    steps_taken_ = 0;
    seen_fresh_ = false;
    start_time_ = ctx_->node->now();
    // Only frames captured after this activation count toward confirmation —
    // a stale cached array (e.g. from before the mission repositioned) must
    // not satisfy the detection instantly.
    std::string camera = "down";
    (void)getInput("camera", camera);
    gate_.seed_from(ctx_->detections_for(camera));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DescendUntilDetected::onRunning()
{
    std::string label = "table", camera = "down";
    double min_conf = 0.40, step_m = 0.20, pos_tol = 0.10;
    int max_steps = 12, timeout_msec = 45000, confirm_frames = 3;
    getInput("label", label);
    getInput("camera", camera);
    getInput("min_conf", min_conf);
    getInput("step_m", step_m);
    getInput("pos_tol", pos_tol);
    getInput("max_steps", max_steps);
    getInput("timeout_msec", timeout_msec);
    getInput("confirm_frames", confirm_frames);

    // Success once the target is seen on confirm_frames consecutive fresh
    // frames (see detection_gate.hpp). Presence and stamp are judged from the
    // SAME array snapshot — fetching twice can pair frame A's presence with
    // frame B's stamp. The miss side of the gate is unused: absence never
    // fails this node — the descent is bounded by max_steps/timeout instead —
    // a fresh miss just resets the hit streak.
    if (auto arr = ctx_->detections_for(camera))
    {
        auto verdict = gate_.update(detection_gate::contains_label(*arr, label, min_conf),
                                    detection_gate::MissGate::stamp_ns_of(*arr), std::numeric_limits<int>::max());
        if (verdict != detection_gate::MissGate::Verdict::kStale)
        {
            seen_fresh_ = true;
        }
        if (verdict == detection_gate::MissGate::Verdict::kHit && gate_.hits >= confirm_frames)
        {
            RCLCPP_INFO(ctx_->logger(), "DescendUntilDetected: '%s' confirmed (%d frames) after %d step(s)",
                        label.c_str(), gate_.hits, steps_taken_);
            return BT::NodeStatus::SUCCESS;
        }
    }

    // Timeout guard — the ABSOLUTE bound (fires even mid-streak; a dead
    // stream mid-confirmation must not hold this node RUNNING forever).
    if ((ctx_->node->now() - start_time_).seconds() * 1000.0 > timeout_msec)
    {
        RCLCPP_WARN(ctx_->logger(), "DescendUntilDetected: timed out without seeing '%s'", label.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Wait until the previous step's goal is reached before issuing the next.
    if (waiting_for_goal_)
    {
        geometry_msgs::msg::Pose cur{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (ctx_->latest_odom)
                cur = ctx_->latest_odom->pose.pose;
        }
        double dx = cur.position.x - pending_goal_.position.x;
        double dy = cur.position.y - pending_goal_.position.y;
        double dz = cur.position.z - pending_goal_.position.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > pos_tol)
            return BT::NodeStatus::RUNNING;
        waiting_for_goal_ = false;
    }

    // Hold depth while confirmation is in progress: a live hit streak means
    // the target is probably right below — descending further would overshoot
    // (and previously guaranteed one extra 0.2 m step that chained into every
    // later relative-depth move via ctx->last_goal). Also hold until the
    // FIRST post-start frame has been evaluated, so a target visible from the
    // start confirms with zero steps (the legacy fast path, minus its
    // stale-cached-frame bug). Both holds are bounded by timeout_msec above.
    if (gate_.hits > 0 || !seen_fresh_)
    {
        return BT::NodeStatus::RUNNING;
    }

    // Out of steps -> give up (bounded so we never drive into the floor).
    // Only reached with no streak in progress, so max_steps can never cut off
    // a confirmation that is about to succeed (the timeout above remains the
    // final arbiter for that corner).
    if (steps_taken_ >= max_steps)
    {
        RCLCPP_WARN(ctx_->logger(), "DescendUntilDetected: reached max_steps (%d) without '%s'", max_steps,
                    label.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Command one downward step (z decreases in ENU), holding XY + orientation.
    geometry_msgs::msg::Pose goal{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            goal = ctx_->latest_odom->pose.pose;
    }
    goal.position.z -= step_m;
    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }
    pending_goal_ = goal;
    waiting_for_goal_ = true;
    ++steps_taken_;
    RCLCPP_INFO(ctx_->logger(), "DescendUntilDetected: step %d -> z=%.2f", steps_taken_, goal.position.z);
    return BT::NodeStatus::RUNNING;
}

void DescendUntilDetected::onHalted()
{
    waiting_for_goal_ = false;
}
