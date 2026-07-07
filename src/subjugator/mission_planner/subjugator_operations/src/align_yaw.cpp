#include "align_yaw.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "detection_gate.hpp"
#include "quat_math.hpp"

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
        BT::InputPort<double>("max_yaw_deg", 12.0, "Max yaw correction per step (deg)"),
        BT::InputPort<double>("ori_tol_deg", 4.0, "Goal-reached orientation tolerance (deg)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus AlignYaw::onStart()
{
    if (!require_ctx(*this, ctx_, "AlignYaw"))
    {
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignYaw::onRunning()
{
    std::string label;
    double min_conf = 0.30, kp = 0.7, yaw_tol_norm = 0.06;
    double max_yaw_deg = 12.0, ori_tol_deg = 4.0;
    getInput("label", label);
    getInput("min_conf", min_conf);
    getInput("kp", kp);
    getInput("yaw_tol_norm", yaw_tol_norm);
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
    uint32_t W = 0, H = 0;
    ctx_->image_size_for("front", W, H);
    if (W == 0)
    {
        RCLCPP_WARN(ctx_->logger(), "AlignYaw: image width unknown, waiting");
        return BT::NodeStatus::RUNNING;
    }

    // Best matching detection
    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for("front");
    auto const* best = arr ? detection_gate::best_detection(*arr, label, min_conf) : nullptr;

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

    // Direct pixel mapping: error_x in [-1,1], scale straight to yaw degrees.
    // error_x > 0 → target right of centre → yaw right → negative cmd (ROS2 CCW-positive)
    double yaw_cmd_deg = -kp * error_x * max_yaw_deg;
    yaw_cmd_deg = std::clamp(yaw_cmd_deg, -max_yaw_deg, max_yaw_deg);

    // Compose: goal_q = current_q * yaw_delta(yaw_cmd_deg)  (same pattern as HoneBearing)
    geometry_msgs::msg::Pose cur{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            cur = ctx_->latest_odom->pose.pose;
    }

    geometry_msgs::msg::Pose goal = cur;  // copy preserves position
    goal.orientation = quat_math::multiply(cur.orientation, quat_math::yaw_delta(yaw_cmd_deg));
    quat_math::normalize(goal.orientation);

    ctx_->command_goal(goal);
    pending_goal_ = goal;
    waiting_for_goal_ = true;

    RCLCPP_INFO(ctx_->logger(), "AlignYaw: error_x=%.3f → yaw_cmd=%.1f°", error_x, yaw_cmd_deg);
    return BT::NodeStatus::RUNNING;
}

void AlignYaw::onHalted()
{
    waiting_for_goal_ = false;
}
