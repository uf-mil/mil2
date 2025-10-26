#include "hone_bearing.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

HoneBearing::HoneBearing(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList HoneBearing::providedPorts()
{
    return { BT::InputPort<std::string>("label", "shark", "Target label (single)"),
             BT::InputPort<double>("offset_deg", 0.0, "Positive=left yaw bias, negative=right"),
             BT::InputPort<double>("tolerance_deg", 3.0, "Acceptable error"),
             BT::InputPort<double>("fov_deg", 90.0, "Horizontal FOV of camera"),
             BT::InputPort<double>("max_step_deg", 10.0, "Max yaw per command"),
             BT::InputPort<double>("min_conf", 0.30, "Minimum confidence"),
             BT::InputPort<std::shared_ptr<Context>>("ctx") };
}

BT::NodeStatus HoneBearing::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "HoneBearing: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoneBearing::onRunning()
{
    std::string label;
    double offset_deg = 0.0, tol = 3.0, fov = 90.0, max_step = 10.0, min_conf = 0.30;

    (void)getInput("label", label);
    (void)getInput("offset_deg", offset_deg);
    (void)getInput("tolerance_deg", tol);
    (void)getInput("fov_deg", fov);
    (void)getInput("max_step_deg", max_step);
    (void)getInput("min_conf", min_conf);

    // image size
    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }
    if (W == 0)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "HoneBearing: image size unknown.");
        return BT::NodeStatus::RUNNING;
    }

    // Find best detection with matching label
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
        return BT::NodeStatus::RUNNING;

    yolo_msgs::msg::Detection const* best = nullptr;
    double best_conf = 0.0;
    for (auto const& det : arr->detections)
    {
        if (det.class_name == label && det.score >= min_conf && det.score > best_conf)
        {
            best = &det;
            best_conf = det.score;
        }
    }

    if (!best)
        return BT::NodeStatus::RUNNING;

    // Pixel -> bearing (deg). Center=0, right positive.
    double cx = best->bbox.center.position.x;
    double bearing = ((cx - (double)W / 2.0) / ((double)W / 2.0)) * (fov / 2.0);

    double desired = offset_deg;

    double err = desired - bearing;
    if (std::fabs(err) <= tol)
        return BT::NodeStatus::SUCCESS;

    double yaw_delta = std::clamp(desired - bearing, -max_step, max_step);

    // Publish relative yaw-only goal: q_new = q_cur * q_delta
    geometry_msgs::msg::Pose current{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            current = ctx_->latest_odom->pose.pose;
    }

    geometry_msgs::msg::Pose out = current;
    double rad = yaw_delta * M_PI / 180.0;

    geometry_msgs::msg::Pose delta;
    delta.orientation.x = 0.0;
    delta.orientation.y = 0.0;
    delta.orientation.z = std::sin(rad / 2.0);
    delta.orientation.w = std::cos(rad / 2.0);

    auto const& c = current.orientation;
    auto const& d = delta.orientation;
    out.orientation.x = c.w * d.x + c.x * d.w + c.y * d.z - c.z * d.y;
    out.orientation.y = c.w * d.y - c.x * d.z + c.y * d.w + c.z * d.x;
    out.orientation.z = c.w * d.z + c.x * d.y - c.y * d.x + c.z * d.w;
    out.orientation.w = c.w * d.w - c.x * d.x - c.y * d.y - c.z * d.z;

    ctx_->goal_pub->publish(out);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = out;
    }

    RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                         "HoneBearing: bearing=%.1f°, desired=%.1f° (offset=%.1f°), yaw_delta=%.1f°", bearing, desired,
                         offset_deg, yaw_delta);

    return BT::NodeStatus::RUNNING;
}

void HoneBearing::onHalted()
{
}
