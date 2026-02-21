#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

BT::PortsList AtGoalPose::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("x"));
    ports.insert(BT::InputPort<double>("y"));
    ports.insert(BT::InputPort<double>("z"));
    ports.insert(BT::InputPort<double>("qx"));
    ports.insert(BT::InputPort<double>("qy"));
    ports.insert(BT::InputPort<double>("qz"));
    ports.insert(BT::InputPort<double>("qw"));
    ports.insert(BT::InputPort<double>("pos_tol", 0.20, "Position tolerance (m)"));
    ports.insert(BT::InputPort<double>("ori_tol_deg", 10.0, "Orientation tolerance (deg)"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx"));
    return ports;
}

class WaypointRecorder : public rclcpp::Node
{
    double gx, gy, gz, gqx, gqy, gqz, gqw, pos_tol, ori_tol_deg;
    if (!getInput("x", gx) || !getInput("y", gy) || !getInput("z", gz) || !getInput("qx", gqx) ||
        !getInput("qy", gqy) || !getInput("qz", gqz) || !getInput("qw", gqw) || !getInput("pos_tol", pos_tol) ||
        !getInput("ori_tol_deg", ori_tol_deg))
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

    auto const& p = odom->pose.pose.position;
    auto const& q = odom->pose.pose.orientation;

    // capture the odometry frame here
};
