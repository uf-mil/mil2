#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <string>

#include "prop_mission_planner/ros_time_budget.hpp"

namespace prop_mission_planner
{

/// The builtin <Timeout>, but its budget elapses in ROS time (node->now())
/// instead of wall time. On the boat the two are the same; in simulation,
/// where this box runs at 0.05-0.9x real time, the builtin would halt a
/// maneuver after a small fraction of the sim-seconds it was given.
///
/// Expiry is noticed on the next tick (10 Hz). A paused sim clock freezes the
/// budget. Adapted from the sub's mission planner (origin/gripper-task-5).
class RosTimeout : public BT::DecoratorNode
{
  public:
    RosTimeout(std::string const &name, BT::NodeConfig const &config);
    static BT::PortsList providedPorts();
    void halt() override;

  private:
    BT::NodeStatus tick() override;

    ros_time_budget::Budget budget_;
};

}  // namespace prop_mission_planner
