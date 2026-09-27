#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace prop_mission_planner
{

/// Makes a stand-in reference to an object at a known map point:
///
///     <StaticObject point="-4;-4" ref="{entry_buoy}"/>
///
/// Once the boat has a map this line becomes <FindObject id=... ref=.../>,
/// and nothing that uses {entry_buoy} changes.
class StaticObject : public BT::SyncActionNode
{
  public:
    StaticObject(std::string const &name, BT::NodeConfig const &config);
    static BT::PortsList providedPorts();

  private:
    BT::NodeStatus tick() override;
};

}  // namespace prop_mission_planner
