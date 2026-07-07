#include "align_depth.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "detection_gate.hpp"

AlignDepth::AlignDepth(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList AlignDepth::providedPorts()
{
    return {
        BT::InputPort<std::string>("label", "torpedo_target", "YOLO class label to track"),
        BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
        BT::InputPort<double>("kp", 0.4, "Proportional gain (0-1)"),
        BT::InputPort<double>("depth_tol_norm", 0.08, "Normalized Y-center error tolerance (0-1)"),
        BT::InputPort<double>("max_step_m", 0.10, "Max depth correction per step (m)"),
        BT::InputPort<double>("pos_tol", 0.10, "Goal-reached position tolerance (m)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus AlignDepth::onStart()
{
    if (!require_ctx(*this, ctx_, "AlignDepth"))
    {
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignDepth::onRunning()
{
    std::string label;
    double min_conf = 0.30, kp = 0.4, depth_tol_norm = 0.08, max_step_m = 0.10, pos_tol = 0.10;
    getInput("label", label);
    getInput("min_conf", min_conf);
    getInput("kp", kp);
    getInput("depth_tol_norm", depth_tol_norm);
    getInput("max_step_m", max_step_m);
    getInput("pos_tol", pos_tol);

    // Block until the sub reaches the previously commanded Z goal before issuing next step
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
        {
            return BT::NodeStatus::RUNNING;
        }
        waiting_for_goal_ = false;
    }

    // Image height required for normalised error
    uint32_t W = 0, H = 0;
    ctx_->image_size_for("front", W, H);
    if (H == 0)
    {
        RCLCPP_WARN(ctx_->logger(), "AlignDepth: image height unknown, waiting");
        return BT::NodeStatus::RUNNING;
    }

    // Best matching detection
    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for("front");
    auto const* best = arr ? detection_gate::best_detection(*arr, label, min_conf) : nullptr;

    if (!best)
    {
        RCLCPP_WARN(ctx_->logger(), "AlignDepth: no '%s' detection, holding position", label.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Normalised Y error: +1 = detection at bottom of image, -1 = at top
    double cy = best->bbox.center.position.y;
    double error_y = (cy - static_cast<double>(H) / 2.0) / (static_cast<double>(H) / 2.0);

    if (std::abs(error_y) < depth_tol_norm)
    {
        RCLCPP_INFO(ctx_->logger(), "AlignDepth: depth aligned (error_y=%.3f)", error_y);
        return BT::NodeStatus::SUCCESS;
    }

    // P step: detection below centre (error_y > 0) → go DOWN → z decreases in ENU (ROS2 world frame)
    double dz = -kp * error_y * max_step_m;
    dz = std::clamp(dz, -max_step_m, max_step_m);

    // Correct only Z; preserve current XY and orientation so nothing drifts
    geometry_msgs::msg::Pose goal{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            goal = ctx_->latest_odom->pose.pose;
    }
    goal.position.z += dz;

    ctx_->command_goal(goal);
    pending_goal_ = goal;
    waiting_for_goal_ = true;

    RCLCPP_INFO(ctx_->logger(), "AlignDepth: error_y=%.3f → dz=%.3fm (target z=%.3f)", error_y, dz, goal.position.z);
    return BT::NodeStatus::RUNNING;
}

void AlignDepth::onHalted()
{
    waiting_for_goal_ = false;
}
