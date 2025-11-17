#pragma once
#include <behaviortree_cpp/bt_factory.h>

class HasFoundPair : public BT::ConditionNode
{
  public:
    HasFoundPair(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
