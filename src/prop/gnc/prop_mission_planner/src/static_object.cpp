#include "prop_mission_planner/static_object.hpp"

#include <rclcpp/rclcpp.hpp>

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
    if (!BT::TreeNode::isBlackboardPointer(config.output_ports.at("ref")))
    {
        throw BT::RuntimeError(name, ": ref must be a blackboard entry, e.g. ref=\"{entry_buoy}\"");
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
    auto const ctx = context_of(*this);
    auto const point = getInput<ObjectRef>("point");
    if (!point)
    {
        RCLCPP_ERROR(ctx->logger(), "%s: %s", name().c_str(), point.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (auto const result = setOutput("ref", *point); !result)
    {
        RCLCPP_ERROR(ctx->logger(), "%s: %s", name().c_str(), result.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace prop_mission_planner
