#pragma once
#include <behaviortree_cpp/decorator_node.h>

#include "context.hpp"
#include "ros_time_budget.hpp"

// Drop-in replacement for the builtin <Delay> whose pause elapses in ROS time
// (ctx->node->now()) instead of steady_clock. Settle pauses exist for PHYSICS
// (gripper attach, wake damping), so they must consume sim-seconds in sim; the
// builtin's wall delay shrinks to nothing at RTF << 1. On the robot
// (use_sim_time=false) behavior is unchanged. RUNNING until the delay elapses,
// then ticks the child and returns its status.
class RosDelay : public BT::DecoratorNode
{
  public:
    RosDelay(std::string const& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    void halt() override;

  private:
    BT::NodeStatus tick() override;

    std::shared_ptr<Context> ctx_;
    ros_time_budget::Budget budget_;
};
