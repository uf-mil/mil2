#include "prop_mission_planner/static_object.hpp"

#include "prop_mission_planner/context.hpp"
#include "prop_mission_planner/object_ref.hpp"
#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

REGISTER(StaticObject)

StaticObject::StaticObject(std::string const &name, BT::NodeConfig const &config) : BT::SyncActionNode(name, config)
{
    // Both ports are required and have no default, so a mission missing one
    // is wrong as written: refuse it when the tree is built.
    if (!port_given(config, "point"))
    {
        throw BT::RuntimeError(name, ": point is required, e.g. point=\"-4.0;-4.0\"");
    }
    if (config.output_ports.count("ref") == 0)
    {
        throw BT::RuntimeError(name, ": ref is required, e.g. ref=\"{entry_buoy}\"");
    }
    if (auto const literal = literal_port(config, "point"))
    {
        (void)BT::convertFromString<ObjectRef>(*literal);  // throws on bad text
    }
}

BT::PortsList StaticObject::providedPorts()
{
    return {
        BT::InputPort<ObjectRef>("point", "Where the object is: \"x;y\" in the map frame"),
        BT::OutputPort<ObjectRef>("ref", "The reference to give maneuver nodes' target port"),
    };
}

BT::NodeStatus StaticObject::tick()
{
    auto const point = getInput<ObjectRef>("point");
    if (!point)
    {
        throw BT::RuntimeError(name(), ": ", point.error());
    }
    (void)setOutput("ref", *point);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace prop_mission_planner
