#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

// Returns SUCCESS when the largest red-pole and white-pole bounding boxes
// meet or exceed the configured size thresholds (in pixels). Otherwise FAILURE.
class PolesBigEnough : public BT::ConditionNode
{
  public:
    PolesBigEnough(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
    int ok_streak_{ 0 };  // consecutive SUCCESS frames required
};
