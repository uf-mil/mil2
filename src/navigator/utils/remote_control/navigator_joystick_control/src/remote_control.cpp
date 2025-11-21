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

#include "navigator_joystick_control/remote_control.h"

namespace navigator_joystick_control
{

RemoteControl::RemoteControl(std::shared_ptr<rclcpp::Node> node, std::string const& name,
                             std::string const& wrench_topic)
  : node_(node), control_name_(name)
{
    // Create wrench publisher
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, 10);

    // Create control signal publishers
    kill_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/kill", 10);
    station_hold_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/station_hold", 10);
    rc_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rc_control", 10);
    emergency_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/emergency_control", 10);
    keyboard_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/keyboard_control", 10);
    auto_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/autonomous_control", 10);
    deploy_thrusters_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/deploy_thrusters", 10);
    retract_thrusters_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/retract_thrusters", 10);
}

void RemoteControl::clear_wrench()
{
    publish_wrench(0.0, 0.0, 0.0, node_->get_clock()->now());
}

void RemoteControl::publish_wrench(double x, double y, double rotation, rclcpp::Time const& stamp)
{
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->header.stamp = stamp;
    msg->header.frame_id = control_name_;
    msg->wrench.force.x = x;
    msg->wrench.force.y = y;
    msg->wrench.torque.z = rotation;
    wrench_pub_->publish(std::move(msg));
}

void RemoteControl::publish_control(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, bool value)
{
    auto msg = std::make_unique<std_msgs::msg::Bool>();
    msg->data = value;
    pub->publish(std::move(msg));
}

void RemoteControl::kill()
{
    publish_control(kill_pub_, true);
}

void RemoteControl::clear_kill()
{
    publish_control(kill_pub_, false);
}

void RemoteControl::station_hold()
{
    publish_control(station_hold_pub_, true);
}

void RemoteControl::select_rc_control()
{
    publish_control(rc_control_pub_, true);
}

void RemoteControl::select_emergency_control()
{
    publish_control(emergency_control_pub_, true);
}

void RemoteControl::select_keyboard_control()
{
    publish_control(keyboard_control_pub_, true);
}

void RemoteControl::select_autonomous_control()
{
    publish_control(auto_control_pub_, true);
}

void RemoteControl::deploy_thrusters()
{
    publish_control(deploy_thrusters_pub_, true);
}

void RemoteControl::retract_thrusters()
{
    publish_control(retract_thrusters_pub_, true);
}

}  // namespace navigator_joystick_control
