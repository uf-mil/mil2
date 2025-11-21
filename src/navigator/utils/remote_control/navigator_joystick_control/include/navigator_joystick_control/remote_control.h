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
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace navigator_joystick_control
{

/**
 * @brief Wrapper for remote control operations.
 *
 * Handles publishing wrench commands, kill/clear kill signals,
 * station hold, and control mode selection.
 */
class RemoteControl
{
  public:
    /**
     * @brief Initialize the RemoteControl wrapper.
     * @param node ROS2 node for creating publishers
     * @param name Control source name (e.g., "joystick")
     * @param wrench_topic Topic to publish wrench commands
     */
    RemoteControl(std::shared_ptr<rclcpp::Node> node, std::string const& name, std::string const& wrench_topic);

    ~RemoteControl() = default;

    /**
     * @brief Clear all wrench commands (set to zero).
     */
    void clear_wrench();

    /**
     * @brief Publish a wrench (force and torque) command.
     * @param x Surge force (forward/backward)
     * @param y Sway force (left/right)
     * @param rotation Yaw torque (rotation)
     * @param stamp Timestamp for the command
     */
    void publish_wrench(double x, double y, double rotation, rclcpp::Time const& stamp);

    /**
     * @brief Send kill signal to disable thrusters.
     */
    void kill();

    /**
     * @brief Clear kill signal to enable thrusters.
     */
    void clear_kill();

    /**
     * @brief Enable station hold mode.
     */
    void station_hold();

    /**
     * @brief Select RC (manual) control mode.
     */
    void select_rc_control();

    /**
     * @brief Select emergency control mode.
     */
    void select_emergency_control();

    /**
     * @brief Select keyboard control mode.
     */
    void select_keyboard_control();

    /**
     * @brief Select autonomous control mode.
     */
    void select_autonomous_control();

    /**
     * @brief Deploy thrusters.
     */
    void deploy_thrusters();

    /**
     * @brief Retract thrusters.
     */
    void retract_thrusters();

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string control_name_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr station_hold_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rc_control_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_control_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr keyboard_control_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_control_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deploy_thrusters_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr retract_thrusters_pub_;

    /**
     * @brief Publish a boolean control message.
     * @param pub Publisher to send on
     * @param value Boolean value to publish
     */
    void publish_control(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, bool value);
};

}  // namespace navigator_joystick_control
