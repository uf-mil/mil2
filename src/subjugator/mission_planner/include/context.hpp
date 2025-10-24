#pragma once
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <mil_msgs/msg/perception_target_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>

struct Context
{
    rclcpp::Node::SharedPtr node;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<mil_msgs::msg::PerceptionTargetArray>::SharedPtr targets_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    // Latest state
    std::mutex odom_mx;
    std::optional<nav_msgs::msg::Odometry> latest_odom;

    std::mutex last_goal_mx;
    std::optional<geometry_msgs::msg::Pose> last_goal;

    std::mutex detections_mx;
    std::optional<mil_msgs::msg::PerceptionTargetArray> latest_targets;

    std::mutex img_mx;
    uint32_t img_width{ 0 };
    uint32_t img_height{ 0 };

    inline rclcpp::Logger logger() const
    {
        return node->get_logger();
    }
};
