#pragma once

#include <behaviortree_cpp/action_node.h>

#include <cstdint>

#include "context.hpp"

// Hands one move to the movement_manager and reports its verdict.
//
// Replaces the PublishGoalPose + AtGoalPose polling pair: frame composition,
// arrival tolerances, the timeout budget and all the per-move logging live in
// the manager now. This leaf only publishes a request and waits for SUCCESS or
// FAILURE - watch /move_status for the detail behind either.
//
// Ports are body-frame when `relative` is true (the default), odom-frame otherwise.
class SendMove final : public BT::StatefulActionNode
{
  public:
    SendMove(std::string const& name, BT::NodeConfiguration const& cfg);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    uint32_t command_id_{ 0 };
};
