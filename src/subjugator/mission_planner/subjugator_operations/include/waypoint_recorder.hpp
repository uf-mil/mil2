#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>

#include "context.hpp"

struct Context;
class OperationBase;

class WaypointRecorder final : public BT::SyncActionNode, public OperationBase
{
  public:
    WaypointRecorder(std::string const& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config), OperationBase(config)
    {
    }

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};
