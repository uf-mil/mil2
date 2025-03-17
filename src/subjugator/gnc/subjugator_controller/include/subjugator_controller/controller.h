#pragma once

#include <control_toolbox/pid.hpp>
#include <iostream>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PIDController : public rclcpp::Node
{
  public:
    PIDController();

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_trajectory_;
};
