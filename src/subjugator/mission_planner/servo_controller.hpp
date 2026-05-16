#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <string>

#include "context.hpp"

class Servocontroller : public BT::SyncActionNode
{
  public:
    Servocontroller(std::string const& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("servo"),             // 0 for dropper and 4 for torpedo
                 BT::InputPort<std::string>("action") };  // first/second for dropper and top/bottom for torpedo
    }

    BT::NodeStatus tick() override;
};
