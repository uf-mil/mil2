#include "descend_until_detected.hpp"

#include <cmath>

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
        BT::InputPort<double>("step_m", 0.20, "Downward step per cycle (m)"),
        BT::InputPort<double>("pos_tol", 0.10, "Goal-reached tolerance (m)"),
        BT::InputPort<int>("max_steps", 12, "Max descend steps before FAILURE"),
        BT::InputPort<int>("timeout_msec", 45000, "Overall timeout (ms)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

bool DescendUntilDetected::label_seen(std::string const& label, std::string const& camera, double min_conf)
{
    auto arr = ctx_->detections_for(camera);
    if (!arr)
        return false;
    for (auto const& d : arr->detections)
        if (d.class_name == label && d.score >= min_conf)
            return true;
    return false;
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
    start_time_ = ctx_->node->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DescendUntilDetected::onRunning()
{
    std::string label = "table", camera = "down";
    double min_conf = 0.40, step_m = 0.20, pos_tol = 0.10;
    int max_steps = 12, timeout_msec = 45000;
    getInput("label", label);
    getInput("camera", camera);
    getInput("min_conf", min_conf);
    getInput("step_m", step_m);
    getInput("pos_tol", pos_tol);
    getInput("max_steps", max_steps);
    getInput("timeout_msec", timeout_msec);

    // Success as soon as the target is visible.
    if (label_seen(label, camera, min_conf))
    {
        RCLCPP_INFO(ctx_->logger(), "DescendUntilDetected: '%s' detected after %d step(s)", label.c_str(),
                    steps_taken_);
        return BT::NodeStatus::SUCCESS;
    }

    // Timeout guard.
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

    // Out of steps -> give up (bounded so we never drive into the floor).
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
