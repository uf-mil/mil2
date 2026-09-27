#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <string>

#include "prop_mission_planner/ros_time_budget.hpp"

namespace prop_mission_planner
{

/// The builtin <Delay>, but the pause elapses in ROS time. Settle pauses exist
/// for physics, so in simulation they must last sim-seconds. RUNNING until the
/// delay has passed, then ticks the child and returns its status.
/// Adapted from the sub's mission planner (origin/gripper-task-5).
class RosDelay : public BT::DecoratorNode
{
  public:
    RosDelay(std::string const &name, BT::NodeConfig const &config);
    static BT::PortsList providedPorts();
    void halt() override;

  private:
    BT::NodeStatus tick() override;

    ros_time_budget::Budget budget_;
};

}  // namespace prop_mission_planner
