// Copyright 2025 University of Florida Machine Intelligence Laboratory
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "navigator_joystick_control/remote_control.h"

#include <sensor_msgs/msg/joy.hpp>

namespace navigator_joystick_control
{

/**
 * @brief ROS2 Joystick control node for the AUV.
 *
 * Subscribes to Joy messages and converts joystick input to control commands
 * via the RemoteControl interface. Handles controller timeout detection and
 * state management.
 *
 * @author Anthony Olive
 * @copyright Copyright 2016, MIL
 * @license MIT
 */
class Joystick : public rclcpp::Node
{
  public:
    /**
     * @brief Initialize the Joystick node.
     */
    explicit Joystick();

    ~Joystick() = default;

  private:
    /**
     * @brief Joy message subscription callback.
     * @param joy The Joy message from the joystick
     */
    void joy_received(sensor_msgs::msg::Joy::SharedPtr const joy);

    /**
     * @brief Check if the joystick controller has timed out.
     * @param joy Current Joy message
     */
    void check_for_timeout(sensor_msgs::msg::Joy::SharedPtr const joy);

    /**
     * @brief Reset the controller state to defaults.
     */
    void reset();

    // Remote control interface
    std::unique_ptr<RemoteControl> remote_;

    // Joystick parameters
    double force_scale_;
    double torque_scale_;

    // State tracking
    bool last_raise_kill_{ false };
    bool last_clear_kill_{ false };
    bool last_station_hold_state_{ false };
    bool last_rc_control_{ false };
    bool last_emergency_control_{ false };
    bool last_keyboard_control_{ false };
    bool last_back_{ false };
    bool last_auto_control_{ false };

    // Thruster deployment tracking
    int thruster_deploy_count_{ 0 };
    int thruster_retract_count_{ 0 };

    // Controller activation tracking
    int start_count_{ 0 };
    sensor_msgs::msg::Joy::SharedPtr last_joy_;
    bool active_{ false };

    // Joy subscription
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

}  // namespace navigator_joystick_control
