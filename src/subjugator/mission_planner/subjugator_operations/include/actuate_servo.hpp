#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "context.hpp"
#include "operations.hpp"

// Passes through request from mission planner to open or close a servo based on input port
// Output port will return status of the servo
class ActuateServo final : public BT::ConditionNode
{
  public:
    ActuateServo(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
    int servo_id_;
    bool gripper_is_open_;  // ID: 1
    bool marble_is_drop_;   // ID: 2
    bool torp_is_launch_;   // ID: 3
};
