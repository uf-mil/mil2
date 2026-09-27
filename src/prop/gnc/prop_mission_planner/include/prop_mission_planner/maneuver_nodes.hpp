#pragma once

#include <memory>
#include <string>

#include "prop_mission_planner/maneuver_node.hpp"

namespace prop_mission_planner
{

/// Turn to face the object.
///     <FaceObject target="{buoy}"/>
class FaceObject : public ManeuverNode
{
  public:
    using ManeuverNode::ManeuverNode;
    static BT::PortsList providedPorts();

  protected:
    std::unique_ptr<prop_maneuvers::Maneuver> make(prop_maneuvers::Context &maneuvers) override;
};

/// Drive up to the object and stop `standoff` metres short of its surface.
///     <ApproachObject target="{buoy}" standoff="1.0"/>
class ApproachObject : public ManeuverNode
{
  public:
    using ManeuverNode::ManeuverNode;
    static BT::PortsList providedPorts();

  protected:
    std::unique_ptr<prop_maneuvers::Maneuver> make(prop_maneuvers::Context &maneuvers) override;
};

/// Circle the object in straight legs.
///     <CircleObject target="{buoy}" direction="clockwise"/>
/// direction is REQUIRED: Task 1 is scored on which way the boat goes round.
class CircleObject : public ManeuverNode
{
  public:
    CircleObject(std::string const &name, BT::NodeConfig const &config);
    static BT::PortsList providedPorts();

  protected:
    std::unique_ptr<prop_maneuvers::Maneuver> make(prop_maneuvers::Context &maneuvers) override;
};

}  // namespace prop_mission_planner
