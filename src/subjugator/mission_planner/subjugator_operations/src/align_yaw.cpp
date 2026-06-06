#include "align_yaw.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AlignYaw::AlignYaw(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList AlignYaw::providedPorts()
{
    return {
        BT::InputPort<std::string>("label", "torpedo_target", "YOLO class label to track"),
        BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
        BT::InputPort<double>("kp", 0.7, "Proportional gain (0-1)"),
        BT::InputPort<double>("yaw_tol_norm", 0.06, "Normalised X-centre error tolerance (0-1)"),
        BT::InputPort<double>("fov_deg", 110.0, "Camera horizontal FOV in degrees"),
        BT::InputPort<double>("max_yaw_deg", 12.0, "Max yaw correction per step (deg)"),
        BT::InputPort<double>("ori_tol_deg", 4.0, "Goal-reached orientation tolerance (deg)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus AlignYaw::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "AlignYaw: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignYaw::onRunning()
{
    std::string label;
    double min_conf = 0.30, kp = 0.7, yaw_tol_norm = 0.06;
    double fov_deg = 110.0, max_yaw_deg = 12.0, ori_tol_deg = 4.0;
    getInput("label", label);
    getInput("min_conf", min_conf);
    getInput("kp", kp);
    getInput("yaw_tol_norm", yaw_tol_norm);
    getInput("fov_deg", fov_deg);
    getInput("max_yaw_deg", max_yaw_deg);
    getInput("ori_tol_deg", ori_tol_deg);

    // Block until sub reaches the previously commanded yaw goal before next step
    if (waiting_for_goal_)
    {
        geometry_msgs::msg::Pose cur{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (ctx_->latest_odom)
                cur = ctx_->latest_odom->pose.pose;
        }
        // Angular error via quaternion dot product
        auto const& a = cur.orientation;
        auto const& b = pending_goal_.orientation;
        double dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
        dot = std::clamp(dot, 0.0, 1.0);
        double err_deg = 2.0 * std::acos(dot) * 180.0 / M_PI;
        if (err_deg > ori_tol_deg)
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
        RCLCPP_WARN(ctx_->logger(), "AlignYaw: image width unknown, waiting");
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
        RCLCPP_WARN(ctx_->logger(), "AlignYaw: no '%s' detection, holding heading", label.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Normalised X error: +1 = detection at right edge, -1 = at left edge
    double cx = best->bbox.center.position.x;
    double error_x = (cx - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0);

    if (std::abs(error_x) < yaw_tol_norm)
    {
        RCLCPP_INFO(ctx_->logger(), "AlignYaw: centred (error_x=%.3f)", error_x);
        return BT::NodeStatus::SUCCESS;
    }

    // Bearing in degrees, right-positive
    double bearing_deg = error_x * (fov_deg / 2.0);

    // P step: target to the right (bearing > 0) → yaw right → negative yaw in ROS2 (CCW positive)
    double yaw_cmd_deg = -kp * bearing_deg;
    yaw_cmd_deg = std::clamp(yaw_cmd_deg, -max_yaw_deg, max_yaw_deg);

    // Compose: goal_q = current_q * delta_q  (same pattern as HoneBearing)
    geometry_msgs::msg::Pose cur{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            cur = ctx_->latest_odom->pose.pose;
    }

    double yaw_rad = yaw_cmd_deg * M_PI / 180.0;
    geometry_msgs::msg::Quaternion dq{};
    dq.z = std::sin(yaw_rad / 2.0);
    dq.w = std::cos(yaw_rad / 2.0);

    geometry_msgs::msg::Pose goal = cur;  // copy preserves position
    auto const& c = cur.orientation;
    auto& o = goal.orientation;
    o.x = c.w * dq.x + c.x * dq.w + c.y * dq.z - c.z * dq.y;
    o.y = c.w * dq.y - c.x * dq.z + c.y * dq.w + c.z * dq.x;
    o.z = c.w * dq.z + c.x * dq.y - c.y * dq.x + c.z * dq.w;
    o.w = c.w * dq.w - c.x * dq.x - c.y * dq.y - c.z * dq.z;

    double n = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    if (n > 1e-12)
    {
        o.x /= n;
        o.y /= n;
        o.z /= n;
        o.w /= n;
    }
    else
    {
        o.x = 0.0;
        o.y = 0.0;
        o.z = 0.0;
        o.w = 1.0;
    }

    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }
    pending_goal_ = goal;
    waiting_for_goal_ = true;

    RCLCPP_INFO(ctx_->logger(), "AlignYaw: error_x=%.3f bearing=%.1f° → cmd=%.1f°", error_x, bearing_deg, yaw_cmd_deg);
    return BT::NodeStatus::RUNNING;
}

void AlignYaw::onHalted()
{
    waiting_for_goal_ = false;
}
