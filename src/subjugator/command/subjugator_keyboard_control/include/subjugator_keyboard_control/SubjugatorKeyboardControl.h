#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/wrench.hpp"
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
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    // Add publisher for keypress events
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keypress_publisher_;
    std::atomic<double> force_x_, force_y_, force_z_;
    std::atomic<double> torque_x_, torque_y_, torque_z_;
    double base_linear_, base_angular_;
    std::thread keyboard_thread_;
    std::thread publisher_thread_;
    std::atomic<bool> running_;
    termios old_terminal_settings_{};
    bool terminal_initialized_{ false };
    void initTerminal();
    void restoreTerminal() const;
    void keyboardLoop();
    void publishLoop() const;
};
