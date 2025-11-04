#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

class AnyPolesDetected : public BT::ConditionNode
{
  public:
    AnyPolesDetected(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
};
