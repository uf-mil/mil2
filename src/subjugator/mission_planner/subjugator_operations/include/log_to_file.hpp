#pragma once
#include <behaviortree_cpp/bt_factory.h>

class LogToFile : public BT::SyncActionNode
{
  public:
    LogToFile(std::string const& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
