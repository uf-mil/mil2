#include "waypoint_recorder.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <mutex>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "context.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "operations.hpp"

BT::NodeStatus WaypointRecorder::tick()
{
    double gx, gy, gz, gqx, gqy, gqz, gqw, spacing_m;
    std::shared_ptr<Context> ctx_;

    if (!getInput("x", gx) || !getInput("y", gy) || !getInput("z", gz) || !getInput("qx", gqx) ||
        !getInput("qy", gqy) || !getInput("qz", gqz) || !getInput("qw", gqw) || !getInput("spacing_m", spacing_m) ||
        !getInput("ctx_", ctx_))
    {
        RCLCPP_ERROR(ctx_->logger(), "WaypointRecorder: missing required inputs.");
    }

    std::optional<nav_msgs::msg::Odometry> odom;
    {
        std::scoped_lock lk(ctx_->odom_mx);
        odom = ctx_->latest_odom;
    }

    if (!odom)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "WaypointRecorder: no odometry yet.");
    }
    else
    {
        geometry_msgs::msg::PoseStamped wp;
        wp.header = odom->header;
        wp.pose = odom->pose.pose;

        std::scoped_lock lkwp(ctx_->waypoints_mx);

        if (!ctx_->last_waypoint)
        {
            ctx_->waypoints.push_back(wp);
            ctx_->last_waypoint = wp;

            if (ctx_)
            {
                RCLCPP_INFO(ctx_->logger(), "WaypointRecorder: first waypoint recorded.");
            }
        }
        else
        {
            auto const& last = ctx_->last_waypoint->pose.position;
            double const dx = wp.pose.position.x - last.x;
            double const dy = wp.pose.position.y - last.y;
            double const d = std::sqrt(dx * dx + dy * dy);

            if (d >= spacing_m)
            {
                ctx_->waypoints.push_back(wp);
                ctx_->last_waypoint = wp;
                RCLCPP_INFO(ctx_->logger(), "WaypointRecorder: recorded waypoint #%zu", ctx_->waypoints.size());
            }
        }
    }
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList WaypointRecorder::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("x"));
    ports.insert(BT::InputPort<double>("y"));
    ports.insert(BT::InputPort<double>("z"));
    ports.insert(BT::InputPort<double>("qx"));
    ports.insert(BT::InputPort<double>("qy"));
    ports.insert(BT::InputPort<double>("qz"));
    ports.insert(BT::InputPort<double>("qw"));
    ports.insert(BT::InputPort<double>("spacing_m", 0.5, "Waypoint spacing in meters"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx_"));
    return ports;
}
