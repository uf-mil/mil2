#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

#include <subjugator_msgs/srv/servo.hpp>

// Calls one of the servo services exposed by servo_controller/driver.py
// (/dropper, /gripper, /torpedo). Async: returns RUNNING until the response
// arrives, then SUCCESS.
class ActuateServo : public BT::StatefulActionNode
{
  public:
    ActuateServo(std::string const& name, const BT::NodeConfiguration& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    using Servo = subjugator_msgs::srv::Servo;

    std::shared_ptr<Context> ctx_;
    rclcpp::Client<Servo>::SharedFuture future_;
    bool in_flight_{ false };
    std::string target_;
};
