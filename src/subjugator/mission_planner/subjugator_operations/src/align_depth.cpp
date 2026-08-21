#include "align_depth.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

REGISTER(AlignDepth)

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
        BT::InputPort<double>("center_offset_px", 0.0,
                              "Desired vertical offset of target from image center (px); + = down"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus AlignDepth::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "AlignDepth: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignDepth::onRunning()
{
    std::string label;
    double min_conf = 0.30, kp = 0.4, depth_tol_norm = 0.08, max_step_m = 0.10, pos_tol = 0.10;
    double center_offset_px = 0.0;
    getInput("label", label);
    getInput("min_conf", min_conf);
    getInput("kp", kp);
    getInput("depth_tol_norm", depth_tol_norm);
    getInput("max_step_m", max_step_m);
    getInput("pos_tol", pos_tol);
    getInput("center_offset_px", center_offset_px);

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
    uint32_t H = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        H = ctx_->img_height;
    }
    if (H == 0)
    {
        RCLCPP_WARN(ctx_->logger(), "AlignDepth: image height unknown, waiting");
        return BT::NodeStatus::RUNNING;
    }

    // Best matching detection
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

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
        RCLCPP_WARN(ctx_->logger(), "AlignDepth: no '%s' detection, holding position", label.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Normalised Y error about the desired image point (center + offset):
    // +1 = detection at bottom of image, -1 = at top
    double cy = best->bbox.center.position.y;
    double const target_y = static_cast<double>(H) / 2.0 + center_offset_px;
    double error_y = (cy - target_y) / (static_cast<double>(H) / 2.0);

    if (std::abs(error_y) < depth_tol_norm)
    {
        RCLCPP_INFO(ctx_->logger(), "AlignDepth: depth aligned (error_y=%.3f)", error_y);
        return BT::NodeStatus::SUCCESS;
    }

    // P step: detection below centre (error_y > 0) → go DOWN → z decreases in ENU (ROS2 world frame)
    double dz = -kp * error_y * max_step_m;
    dz = std::clamp(dz, -max_step_m, max_step_m);

    // Chain the Z correction off the LAST COMMANDED GOAL (same pattern as RelativeMove /
    // BoardArchStep) so per-step station-keeping error does not accumulate across steps.
    // XY and orientation carry through unchanged; only Z is adjusted. Falls back to live
    // odom for the very first goal, before anything has been commanded.
    geometry_msgs::msg::Pose base{};
    bool have_base = false;
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        if (ctx_->last_goal)
        {
            base = *ctx_->last_goal;
            have_base = true;
        }
    }
    if (!have_base)
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            base = ctx_->latest_odom->pose.pose;
        else
            base.orientation.w = 1.0;
    }

    geometry_msgs::msg::Pose goal = base;
    goal.position.z = base.position.z + dz;

    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }
    pending_goal_ = goal;
    waiting_for_goal_ = true;

    RCLCPP_INFO(ctx_->logger(), "AlignDepth: error_y=%.3f → dz=%.3fm (target z=%.3f)", error_y, dz, goal.position.z);
    return BT::NodeStatus::RUNNING;
}

void AlignDepth::onHalted()
{
    waiting_for_goal_ = false;
}
