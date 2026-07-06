#pragma once
#include <behaviortree_cpp/decorator_node.h>

#include "context.hpp"
#include "ros_time_budget.hpp"

// Drop-in replacement for the builtin <Timeout> whose budget elapses in ROS
// time (ctx->node->now()) instead of steady_clock. On the robot the two tick
// together, so pool behavior is unchanged; under use_sim_time=true the budget
// buys the mission the intended SIM-seconds of motion even at RTF << 1, where
// the builtin's wall budget starves motion 20-50x (custom nodes' timeout_msec
// ports already work this way — this closes the gap for the XML decorators).
//
// Differences from the builtin worth knowing:
//  - Tick-driven: expiry is noticed on the next tree tick (30 Hz main loop),
//    not by a wall-timer thread, so latency is one tick period.
//  - A paused sim clock freezes the budget: the child stays RUNNING instead
//    of being failed while physics is not advancing.
class RosTimeout : public BT::DecoratorNode
{
  public:
    RosTimeout(std::string const& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    void halt() override;

  private:
    BT::NodeStatus tick() override;

    std::shared_ptr<Context> ctx_;
    ros_time_budget::Budget budget_;
};
