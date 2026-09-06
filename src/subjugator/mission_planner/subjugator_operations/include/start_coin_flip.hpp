#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <memory>
#include <string>

#include "context.hpp"

// Kills the coin_flip_node child (if any) and reaps it. Safe to call when nothing
// was launched. Shared by StopCoinFlip and the mission planner's shutdown cleanup
// so the two stay in sync.
void stopCoinFlip(Context& ctx);

// Launches the coin_flip classifier node (subjugator_vision) as a child process,
// then blocks until it is actually publishing on /coin_flip/direction (or times
// out). The PID is stored in the Context so it can be killed on shutdown.
// Idempotent: if already launched, just re-confirms it is publishing.
class StartCoinFlip : public BT::StatefulActionNode
{
  public:
    StartCoinFlip(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    bool launchNode();  // spawn python3 <coin_flip_node.py>; caller holds child_mx

    std::shared_ptr<Context> ctx_;
    double timeout_s_{ 30.0 };
    std::chrono::steady_clock::time_point start_time_;
};
