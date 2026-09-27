#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <optional>
#include <string>

#include "prop_maneuvers/maneuvers.hpp"
#include "prop_mission_planner/context.hpp"
#include "prop_mission_planner/object_ref.hpp"
#include "prop_mission_planner/ros_time_budget.hpp"

namespace prop_mission_planner
{

/// Runs one prop_maneuvers maneuver as a behavior-tree action.
///
/// Subclasses only say which maneuver to build. Locking on, stepping, and
/// stopping the motors when interrupted or failed all live here, once:
///
///   onStart   release the previous lock, then try to lock on
///   onRunning keep trying to lock until lock_timeout, then step the maneuver
///   onHalted  release guidance, stop spinner and reverser, drop the lock
///
/// On SUCCESS nothing is released: the boat is left exactly as the maneuver
/// left it, the same as the standalone programs.
class ManeuverNode : public BT::StatefulActionNode
{
  public:
    ManeuverNode(std::string const &name, BT::NodeConfig const &config);

    /// Ports every maneuver node has. Subclasses append their own.
    static BT::PortsList common_ports();

  protected:
    /// Build the maneuver. Called once per run, right after the lock is on.
    virtual std::unique_ptr<prop_maneuvers::Maneuver> make(prop_maneuvers::Context &maneuvers) = 0;

  private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    BT::NodeStatus try_lock();
    BT::NodeStatus step();
    void stop_all();

    std::shared_ptr<Context> ctx_;
    std::optional<ObjectRef> target_;
    bool in_front_{ false };
    double lock_timeout_{ 0.0 };
    ros_time_budget::Budget lock_budget_;
    std::unique_ptr<prop_maneuvers::Maneuver> maneuver_;
};

}  // namespace prop_mission_planner
