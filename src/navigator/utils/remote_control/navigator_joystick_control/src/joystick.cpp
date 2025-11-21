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

#include "navigator_joystick_control/joystick.h"

namespace navigator_joystick_control
{

Joystick::Joystick() : rclcpp::Node("joystick")
{
    // Get parameters with defaults
    this->declare_parameter<double>("force_scale", 600.0);
    this->declare_parameter<double>("torque_scale", 500.0);

    force_scale_ = this->get_parameter("force_scale").as_double();
    torque_scale_ = this->get_parameter("torque_scale").as_double();

    // Initialize remote control interface
    remote_ = std::make_unique<RemoteControl>(shared_from_this(), "joystick", "/wrench/rc");

    // Initialize state
    reset();

    // Subscribe to joy messages
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr const msg) { this->joy_received(msg); });

    RCLCPP_INFO(this->get_logger(), "Joystick control node initialized");
}

void Joystick::reset()
{
    last_raise_kill_ = false;
    last_clear_kill_ = false;
    last_station_hold_state_ = false;
    last_rc_control_ = false;
    last_emergency_control_ = false;
    last_keyboard_control_ = false;
    last_back_ = false;
    last_auto_control_ = false;

    thruster_deploy_count_ = 0;
    thruster_retract_count_ = 0;

    start_count_ = 0;
    last_joy_ = nullptr;
    active_ = false;

    remote_->clear_wrench();
}

void Joystick::check_for_timeout(sensor_msgs::msg::Joy::SharedPtr const joy)
{
    if (!last_joy_)
    {
        last_joy_ = joy;
        return;
    }

    // No change in state - controller may have timed out after 15 minutes
    if (joy->axes == last_joy_->axes && joy->buttons == last_joy_->buttons &&
        this->now() - last_joy_->header.stamp > rclcpp::Duration(15 * 60, 0) && active_)
    {
        RCLCPP_WARN(this->get_logger(), "Controller Timed out. Hold start to resume.");
        reset();
    }
    else
    {
        // Update timestamp to current time
        last_joy_ = joy;
        last_joy_->header.stamp = this->now();
    }
}

void Joystick::joy_received(sensor_msgs::msg::Joy::SharedPtr const joy)
{
    check_for_timeout(joy);

    // Assign readable names to buttons
    bool start = joy->buttons[7];
    bool back = joy->buttons[6];
    bool raise_kill = static_cast<bool>(joy->buttons[1]);    // B
    bool clear_kill = static_cast<bool>(joy->buttons[2]);    // X
    bool station_hold = static_cast<bool>(joy->buttons[0]);  // A
    bool rc_control = joy->axes[6] > 0.9;                    // d-pad left
    bool emergency_control = false;
    bool keyboard_control = false;
    bool auto_control = joy->axes[6] < -0.9;                     // d-pad right
    bool thruster_retract = static_cast<bool>(joy->buttons[4]);  // LB
    bool thruster_deploy = static_cast<bool>(joy->buttons[5]);   // RB

    // Back button to go inactive
    if (back && !last_back_)
    {
        RCLCPP_INFO(this->get_logger(), "Back pressed. Going inactive");
        reset();
        return;
    }

    // Start button (about 1 second = 10+ callbacks at ~10Hz)
    start_count_ += start ? 1 : 0;
    if (start_count_ > 5)
    {
        RCLCPP_INFO(this->get_logger(), "Resetting controller state");
        reset();
        active_ = true;
    }

    if (!active_)
    {
        remote_->clear_wrench();
        return;
    }

    // Track thruster button presses
    if (thruster_retract)
    {
        thruster_retract_count_++;
    }
    else
    {
        thruster_retract_count_ = 0;
    }

    if (thruster_deploy)
    {
        thruster_deploy_count_++;
    }
    else
    {
        thruster_deploy_count_ = 0;
    }

    // Execute thruster commands when button held long enough
    if (thruster_retract_count_ > 10)
    {
        remote_->retract_thrusters();
        thruster_retract_count_ = 0;
    }
    else if (thruster_deploy_count_ > 10)
    {
        remote_->deploy_thrusters();
        thruster_deploy_count_ = 0;
    }

    // Handle control commands (on rising edge of button)
    if (raise_kill && !last_raise_kill_)
    {
        remote_->kill();
    }

    if (clear_kill && !last_clear_kill_)
    {
        remote_->clear_kill();
    }

    if (station_hold && !last_station_hold_state_)
    {
        remote_->station_hold();
    }

    if (rc_control && !last_rc_control_)
    {
        remote_->select_rc_control();
    }

    if (emergency_control && !last_emergency_control_)
    {
        remote_->select_emergency_control();
    }

    if (keyboard_control && !last_keyboard_control_)
    {
        remote_->select_keyboard_control();
    }

    if (auto_control && !last_auto_control_)
    {
        remote_->select_autonomous_control();
    }

    // Update state for next iteration
    last_back_ = back;
    last_raise_kill_ = raise_kill;
    last_clear_kill_ = clear_kill;
    last_station_hold_state_ = station_hold;
    last_rc_control_ = rc_control;
    last_emergency_control_ = emergency_control;
    last_keyboard_control_ = keyboard_control;
    last_auto_control_ = auto_control;

    // Scale joystick input to force and publish wrench
    double x = joy->axes[1] * force_scale_;
    double y = joy->axes[0] * force_scale_;
    double rotation = joy->axes[3] * torque_scale_;
    remote_->publish_wrench(x, y, rotation, joy->header.stamp);
}

}  // namespace navigator_joystick_control
