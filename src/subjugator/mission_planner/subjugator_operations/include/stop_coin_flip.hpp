#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Stops the coin_flip classifier node launched by StartCoinFlip. Always succeeds
// (no-op if nothing is running). Use it to tear the node down mid-mission; the
// mission planner also stops it automatically on shutdown.
class StopCoinFlip : public BT::SyncActionNode
{
  public:
    StopCoinFlip(std::string const& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context") };
    }

    BT::NodeStatus tick() override;
};
