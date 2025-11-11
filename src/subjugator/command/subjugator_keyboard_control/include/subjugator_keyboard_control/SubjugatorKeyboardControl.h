#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

// May be used elsewhere if similar logic?
struct KeyState
{
    bool pressed = false;
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
};

struct AxisEffect
{
    KeyState* key;
    double multiplier;
    double* target;  // What we're updating
};

class SubjugatorKeyboardControl final : public rclcpp::Node
{
  public:
    SubjugatorKeyboardControl();
    ~SubjugatorKeyboardControl() override;

  private:
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    // Add publisher for keypress events
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keypress_publisher_;
    void odometryCallback(nav_msgs::msg::Odometry::SharedPtr const msg);
    std::atomic<double> force_x_, force_y_, force_z_;
    std::atomic<double> torque_x_, torque_y_, torque_z_;
    double base_linear_, base_angular_;
    std::thread keyboard_thread_;
    std::thread publisher_thread_;
    std::atomic<bool> running_;
    termios old_terminal_settings_{};
    nav_msgs::msg::Odometry last_odom_;
    bool terminal_initialized_{ false };
    bool use_smart_control_{ false };
    void initTerminal();
    void restoreTerminal() const;
    void keyboardLoop();
    void publishLoop() const;
    geometry_msgs::msg::Pose createGoalPose() const;
    static geometry_msgs::msg::Pose rotateVectorByQuat(nav_msgs::msg::Odometry const& ref, double rx, double ry,
                                                       double rz);
};
