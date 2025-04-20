#pragma once

#include <SDL2/SDL.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/wrench.hpp"

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
    // Hidden window for key events
    SDL_Window* window_{ nullptr };

    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;

    std::atomic<double> force_x_, force_y_, force_z_;
    std::atomic<double> torque_x_, torque_y_, torque_z_;
    double base_linear_, base_angular_;  // Base speed

    // Multithreading
    std::thread keyboard_thread_;
    std::thread publisher_thread_;
    std::atomic<bool> running_;

    // For handling raw mode terminal
    termios old_tio_{};
    bool termios_initialized_{ false };

    void initTerminal();
    void restoreTerminal() const;

    void keyboardLoop();
    void publishLoop() const;
};
