#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"

// Velocity commands to thruster efforts.
//
//   in   cmd_vel                   geometry_msgs/Twist
//        odometry/filtered/global  nav_msgs/Odometry
//   out  thrusters/left, thrusters/right  std_msgs/Float32, -1 to 1
//        thrusters/heartbeat              std_msgs/Empty
//
// Saturation gives yaw priority: when the pair cannot deliver both, surge is
// given up so the commanded turn survives. The heartbeat is only sent while
// cmd_vel is fresh, so a stalled controller lets the boat's failsafe stop it.
class ThrusterManager : public rclcpp::Node
{
  public:
    ThrusterManager();

  private:
    void step();
    void publish(double left, double right);

    double rate_;
    double command_timeout_;  // s of cmd_vel silence before thrust is dropped
    double max_thrust_;       // N per thruster, must match the boat
    double thruster_y_;       // m off the centreline
    double kp_surge_;
    double ki_surge_;
    double kp_yaw_;
    double ki_yaw_;
    double surge_limit_;  // N, both thrusters at max
    double yaw_limit_;    // N m, both thrusters opposed at max

    geometry_msgs::msg::Twist command_;
    double stamp_{ 0.0 };
    double surge_{ 0.0 };
    double yaw_rate_{ 0.0 };
    double surge_integral_{ 0.0 };
    double yaw_integral_{ 0.0 };

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr beat_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
