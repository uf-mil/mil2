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

    // Sends the request and transitions to the in-flight (RUNNING) state.
    BT::NodeStatus sendRequest();

    std::shared_ptr<Context> ctx_;
    rclcpp::Client<Servo>::SharedPtr client_;
    rclcpp::Client<Servo>::SharedFuture future_;
    bool in_flight_{ false };
    bool awaiting_service_{ false };  // waiting for the service to be discovered
    rclcpp::Time deadline_;           // when to give up waiting for the service
    uint16_t angle_{ 0 };             // validated command, sent once the service is ready
    int service_timeout_msec_{ 3000 };
    std::string target_;
};
