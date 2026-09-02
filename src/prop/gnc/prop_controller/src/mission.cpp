#include "prop_controller/mission.hpp"

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

Mission::Mission() : Node("mission")
{
    auto const waypoints = declare_parameter("waypoints", std::vector<double>{});
    auto const frame = declare_parameter<std::string>("frame_id", "map");

    if (waypoints.empty() || waypoints.size() % 2 != 0)
    {
        RCLCPP_ERROR(get_logger(),
                     "expected a flat list of x, y pairs in \"waypoints\", got %zu values. Launch this node with "
                     "ros2 launch so a mission file is loaded.",
                     waypoints.size());
        return;
    }

    rclcpp::QoS latched(1);
    latched.transient_local();
    publisher_ = create_publisher<nav_msgs::msg::Path>("plan", latched);

    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = now();

    for (std::size_t i = 0; i < waypoints.size(); i += 2)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = waypoints[i];
        pose.pose.position.y = waypoints[i + 1];
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    publisher_->publish(path);
    RCLCPP_INFO(get_logger(), "published a plan of %zu waypoints", path.poses.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mission>());
    rclcpp::shutdown();
    return 0;
}
