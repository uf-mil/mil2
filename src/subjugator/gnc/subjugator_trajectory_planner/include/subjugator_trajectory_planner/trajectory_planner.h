#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// TODO:
// need a subscriber to path/list of poses (perhaps nav_msgs/Path)
// need a subscriber to odom
// need a publisher to goal/trajectory (absolute)

// design plans:
// helper function to calculate closest pose in path?
// param to configure the 'acceptable' distance to goal before publishing the next one

// questions:
// how to search for closest pose, or start at beginning?
// how to handle new paths coming in: stop and do that instead?

class TrajectoryPlanner : public rclcpp::Node
{
  public:
    TrajectoryPlanner();
    void handle_paths();
    void odom_cb(nav_msgs::msg::Odometry::UniquePtr msg);
    void path_cb(nav_msgs::msg::Path::UniquePtr msg);

  private:
    bool heard_newer_path_;
    bool heard_odom_;
    double goal_tolerance_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_publisher_;

    nav_msgs::msg::Odometry::UniquePtr odom_;
    nav_msgs::msg::Path::UniquePtr path_;
};
