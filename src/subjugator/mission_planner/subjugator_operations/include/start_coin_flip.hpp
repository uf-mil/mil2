#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <memory>
#include <string>

#include "context.hpp"

// Kills the coin_flip_node child (if any) by signalling its process group, then
// reaping it. Safe to call when nothing was launched. Shared by StopCoinFlip and
// the mission planner's shutdown cleanup so the two stay in sync.
void stopCoinFlip(Context& ctx);

// Launches the coin_flip classifier node (subjugator_vision) as a child process
// in its own process group, then blocks until it is actually publishing on
// /coin_flip/direction (or times out). The PID is stored in the Context so the
// mission planner kills it on shutdown. Idempotent: if already launched, just
// re-confirms it is publishing.
class StartCoinFlip : public BT::StatefulActionNode
{
  public:
    StartCoinFlip(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    double timeout_s_{ 30.0 };
    std::chrono::steady_clock::time_point start_time_;
};
