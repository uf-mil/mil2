#include "prop_maneuvers/driver.hpp"

#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace prop_maneuvers
{

Driver::Driver(rclcpp::Node *node, double arrive_tolerance) : node_(node), arrive_tolerance_(arrive_tolerance)
{
    // Latched, matching guidance's own subscription, so a plan published once
    // is still there for a guidance node that starts late.
    rclcpp::QoS latched(1);
    latched.transient_local();
    plan_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("plan", latched);
}

void Driver::go_to(std::vector<Point> const &points, std::optional<double> final_heading)
{
    points_ = points;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        Point const &point = points[i];
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        // A zero-length quaternion tells guidance "finish on any heading"; the msg default (w = 1) means yaw 0.
        pose.pose.orientation.w = 0.0;
        if (final_heading && i + 1 == points.size())
        {
            YawQuaternion const q = yaw_quaternion(*final_heading);
            pose.pose.orientation.z = q.z;
            pose.pose.orientation.w = q.w;
        }
        path.poses.push_back(pose);
    }

    plan_publisher_->publish(path);
    Point const last = points.empty() ? Point{} : points.back();
    if (final_heading && points.size() == 1)
    {
        RCLCPP_INFO(node_->get_logger(), "turning to %.0f deg", *final_heading * 180.0 / M_PI);
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "driving %zu point(s), last at (%.1f, %.1f)", points.size(), last.x, last.y);
}

void Driver::go_to(Point const &point, std::optional<double> final_heading)
{
    go_to(std::vector<Point>{ point }, final_heading);
}

void Driver::turn_to(Point const &boat, double heading)
{
    go_to(boat, heading);
}

void Driver::release()
{
    points_.clear();

    // An empty path clears guidance's waypoint list, which makes its timer
    // return before publishing anything. This is what frees cmd_vel for the
    // reverser. Skipping it makes the boat stutter as the two interleave.
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp = node_->now();
    plan_publisher_->publish(empty);

    RCLCPP_INFO(node_->get_logger(), "released guidance");
}

bool Driver::arrived(Point const &boat) const
{
    if (points_.empty())
    {
        return false;
    }
    return distance(boat, points_.back()) <= arrive_tolerance_;
}

}  // namespace prop_maneuvers
