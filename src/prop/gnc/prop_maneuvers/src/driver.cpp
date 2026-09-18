#include "prop_maneuvers/driver.hpp"

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

void Driver::go_to(std::vector<Point> const &points)
{
    points_ = points;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (auto const &point : points)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    plan_publisher_->publish(path);
    RCLCPP_INFO(node_->get_logger(), "driving %zu point(s), last at (%.1f, %.1f)", points.size(),
                points.empty() ? 0.0 : points.back().x, points.empty() ? 0.0 : points.back().y);
}

void Driver::go_to(Point const &point)
{
    go_to(std::vector<Point>{ point });
}

void Driver::release()
{
    points_.clear();

    // An empty path clears guidance's waypoint list, which makes its timer
    // return before publishing anything. This is what frees cmd_vel for the
    // spinner. Skipping it makes the boat stutter as the two interleave.
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
