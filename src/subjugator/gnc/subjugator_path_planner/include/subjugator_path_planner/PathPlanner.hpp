#pragma once

#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPlanner : public rclcpp::Node
{
    int segment_count_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Odometry last_odom_;

  public:
    PathPlanner();
    void goal_pose_cb(geometry_msgs::msg::Pose::SharedPtr const &msg);
    void odom_cb(nav_msgs::msg::Odometry::SharedPtr const &msg);
    auto slerp(geometry_msgs::msg::Pose const &goal_pose) -> std::vector<geometry_msgs::msg::Pose>;
    auto generate_path(geometry_msgs::msg::Pose const &goal_pose) -> std::vector<geometry_msgs::msg::Pose>;
};
