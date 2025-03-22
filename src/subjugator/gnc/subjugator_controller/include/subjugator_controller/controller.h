#pragma once

#include <control_toolbox/pid.hpp>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PIDController : public rclcpp::Node
{
  public:
    PIDController();
    void odom_cb(nav_msgs::msg::Odometry::UniquePtr const msg);
    void goal_trajectory_cb(geometry_msgs::msg::Pose::UniquePtr const msg);

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_goal_trajectory_;
    nav_msgs::msg::Odometry last_odom_;
    geometry_msgs::msg::Pose last_goal_trajectory_;
    control_toolbox::Pid pid_;
    void control_loop();
    rclcpp::Time last_cmd_time_;
};
