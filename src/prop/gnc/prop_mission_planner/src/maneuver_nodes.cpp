#include "prop_mission_planner/maneuver_nodes.hpp"

#include "prop_mission_planner/context.hpp"
#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

namespace
{
/// true for counter-clockwise, as prop_maneuvers::CircleObject wants it.
bool counter_clockwise_from(std::string const &direction)
{
    if (direction == "clockwise")
    {
        return false;
    }
    if (direction == "counter_clockwise")
    {
        return true;
    }
    throw BT::RuntimeError("direction must be \"clockwise\" or \"counter_clockwise\", not \"", direction, "\"");
}
}  // namespace

BT::PortsList FaceObject::providedPorts()
{
    return common_ports();
}

std::unique_ptr<prop_maneuvers::Maneuver> FaceObject::make(prop_maneuvers::Context &maneuvers)
{
    return std::make_unique<prop_maneuvers::FaceObject>(maneuvers);
}

BT::PortsList ApproachObject::providedPorts()
{
    auto ports = common_ports();
    ports.insert(BT::InputPort<double>("standoff", "Metres from the bow to the object's surface. Default: "
                                                   "approach_standoff in maneuvers.yaml"));
    return ports;
}

std::unique_ptr<prop_maneuvers::Maneuver> ApproachObject::make(prop_maneuvers::Context &maneuvers)
{
    double const standoff = getInput<double>("standoff").value_or(maneuvers.settings.approach_standoff_);
    return std::make_unique<prop_maneuvers::ApproachObject>(maneuvers, standoff);
}

CircleObject::CircleObject(std::string const &name, BT::NodeConfig const &config) : ManeuverNode(name, config)
{
    // No default on purpose: a mission that forgets the direction fails to
    // load instead of quietly taking maneuvers.yaml's counter-clockwise.
    if (!port_given(config, "direction"))
    {
        throw BT::RuntimeError(name, ": direction is required (\"clockwise\" or \"counter_clockwise\")");
    }
    if (auto const literal = literal_port(config, "direction"))
    {
        (void)counter_clockwise_from(*literal);  // throws on anything else
    }
}

BT::PortsList CircleObject::providedPorts()
{
    auto ports = common_ports();
    ports.insert(BT::InputPort<std::string>("direction", "\"clockwise\" or \"counter_clockwise\" (required)"));
    ports.insert(BT::InputPort<double>("radius", "Metres. Default: circle_radius in maneuvers.yaml"));
    ports.insert(BT::InputPort<int>("legs", "Straight legs per lap. Default: circle_legs in maneuvers.yaml"));
    return ports;
}

std::unique_ptr<prop_maneuvers::Maneuver> CircleObject::make(prop_maneuvers::Context &maneuvers)
{
    auto const direction = getInput<std::string>("direction");
    if (!direction)
    {
        throw BT::RuntimeError(name(), ": direction: ", direction.error());
    }
    double const radius = getInput<double>("radius").value_or(maneuvers.settings.circle_radius_);
    int const legs = getInput<int>("legs").value_or(maneuvers.settings.circle_legs_);
    return std::make_unique<prop_maneuvers::CircleObject>(maneuvers, radius, legs, counter_clockwise_from(*direction));
}

// Last, after everything the nodes above use (see REGISTER in context.hpp).
REGISTER(FaceObject)
REGISTER(ApproachObject)
REGISTER(CircleObject)

}  // namespace prop_mission_planner
