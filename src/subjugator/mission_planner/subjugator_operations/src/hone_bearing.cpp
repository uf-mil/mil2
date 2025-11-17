#include "hone_bearing.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

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
             BT::InputPort<double>("fov_deg", 110.0, "Horizontal FOV of camera"),
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
    published_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoneBearing::onRunning()
{
    if (published_)
    {
        return BT::NodeStatus::SUCCESS;
    }

    // Inputs
    std::string label;
    double offset_deg = 0.0;
    double fov = 110.0;
    double min_conf = 0.30;

    (void)getInput("label", label);
    (void)getInput("offset_deg", offset_deg);
    (void)getInput("fov_deg", fov);
    (void)getInput("min_conf", min_conf);

    // Get image width
    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }

    if (W == 0)
    {
        RCLCPP_WARN(ctx_->logger(), "HoneBearing: image size unknown");
        return BT::NodeStatus::FAILURE;
    }

    // Best detection
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    if (!arr || arr->detections.empty())
    {
        return BT::NodeStatus::FAILURE;
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

    if (!best)
    {
        return BT::NodeStatus::FAILURE;
    }

    // Pixel -> bearing (deg). Right positive.
    double cx = best->bbox.center.position.x;
    double bearing = ((cx - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0)) * (fov / 2.0);

    // Turn command (deg). Left positive.
    double yaw_cmd_deg = offset_deg - bearing;

    // Compose absolute target orientation: q_out = q_cur * q_delta
    geometry_msgs::msg::Pose current{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
        {
            current = ctx_->latest_odom->pose.pose;
        }
    }

    double yaw_rad = yaw_cmd_deg * M_PI / 180.0;

    geometry_msgs::msg::Quaternion delta_q{};
    delta_q.x = 0.0;
    delta_q.y = 0.0;
    delta_q.z = std::sin(yaw_rad / 2.0);
    delta_q.w = std::cos(yaw_rad / 2.0);

    auto const& c = current.orientation;
    geometry_msgs::msg::Pose out = current;
    auto& o = out.orientation;
    o.x = c.w * delta_q.x + c.x * delta_q.w + c.y * delta_q.z - c.z * delta_q.y;
    o.y = c.w * delta_q.y - c.x * delta_q.z + c.y * delta_q.w + c.z * delta_q.x;
    o.z = c.w * delta_q.z + c.x * delta_q.y - c.y * delta_q.x + c.z * delta_q.w;
    o.w = c.w * delta_q.w - c.x * delta_q.x - c.y * delta_q.y - c.z * delta_q.z;

    // Normalize
    double n = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    if (n < 1e-12)
    {
        o = geometry_msgs::msg::Quaternion{};
        o.w = 1.0;
    }
    else
    {
        o.x /= n;
        o.y /= n;
        o.z /= n;
        o.w /= n;
    }

    // Publish exactly once
    ctx_->goal_pub->publish(out);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = out;
    }
    published_ = true;

    RCLCPP_INFO(ctx_->logger(), "HoneBearing: bearing=%.1f°, offset=%.1f°, cmd=%.1f°", bearing, offset_deg,
                yaw_cmd_deg);

    return BT::NodeStatus::SUCCESS;
}

void HoneBearing::onHalted()
{
}
