#include "align_strafe.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

AlignStrafe::AlignStrafe(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList AlignStrafe::providedPorts()
{
    return {
        BT::InputPort<std::string>("label", "torpedo_target", "YOLO class label to track"),
        BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
        BT::InputPort<double>("kp", 0.5, "Proportional gain (0-1)"),
        BT::InputPort<double>("strafe_tol_norm", 0.08, "Normalized X-center error tolerance (0-1)"),
        BT::InputPort<double>("max_step_m", 0.20, "Max sideways correction per step (m)"),
        BT::InputPort<double>("pos_tol", 0.10, "Goal-reached position tolerance (m)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus AlignStrafe::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "AlignStrafe: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignStrafe::onRunning()
{
    std::string label;
    double min_conf = 0.30, kp = 0.5, strafe_tol_norm = 0.08, max_step_m = 0.20, pos_tol = 0.10;
    getInput("label", label);
    getInput("min_conf", min_conf);
    getInput("kp", kp);
    getInput("strafe_tol_norm", strafe_tol_norm);
    getInput("max_step_m", max_step_m);
    getInput("pos_tol", pos_tol);

    // Block until the sub reaches the previously commanded goal before issuing next step
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

    // Image width required for normalised error
    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }
    if (W == 0)
    {
        RCLCPP_WARN(ctx_->logger(), "AlignStrafe: image width unknown, waiting");
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
        RCLCPP_WARN(ctx_->logger(), "AlignStrafe: no '%s' detection, holding position", label.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Normalised X error: +1 = detection at right edge, -1 = at left edge
    double cx = best->bbox.center.position.x;
    double error_x = (cx - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0);

    if (std::abs(error_x) < strafe_tol_norm)
    {
        RCLCPP_INFO(ctx_->logger(), "AlignStrafe: centred (error_x=%.3f)", error_x);
        return BT::NodeStatus::SUCCESS;
    }

    // P step: target right of centre (error_x > 0) → strafe right → body-frame
    // dy negative (ROS2 body +Y is left)
    double dy_body = -kp * error_x * max_step_m;
    dy_body = std::clamp(dy_body, -max_step_m, max_step_m);

    // Translate only; preserve current orientation and depth
    geometry_msgs::msg::Pose goal{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            goal = ctx_->latest_odom->pose.pose;
    }

    // Rotate the body-frame Y step into the world frame using current yaw
    auto const& q = goal.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    goal.position.x += -std::sin(yaw) * dy_body;
    goal.position.y += std::cos(yaw) * dy_body;

    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }
    pending_goal_ = goal;
    waiting_for_goal_ = true;

    RCLCPP_INFO(ctx_->logger(), "AlignStrafe: error_x=%.3f → dy=%.3fm", error_x, dy_body);
    return BT::NodeStatus::RUNNING;
}

void AlignStrafe::onHalted()
{
    waiting_for_goal_ = false;
}
