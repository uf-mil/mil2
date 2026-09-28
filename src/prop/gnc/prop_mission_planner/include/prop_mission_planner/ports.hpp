#pragma once

#include <behaviortree_cpp/tree_node.h>

#include <cstdint>
#include <optional>
#include <string>

#include "prop_mission_planner/ros_time_budget.hpp"

namespace prop_mission_planner
{

/// The text a port was given in the XML, when it was a literal rather than a
/// {blackboard} entry. Lets a node reject a bad mission when the tree is built
/// instead of halfway through a run.
///
/// Careful: BT.CPP fills in a port's DEFAULT when the XML leaves it out, so
/// "not given" can only be detected for ports declared without a default.
inline std::optional<std::string> literal_port(BT::NodeConfig const &config, std::string const &key)
{
    auto const it = config.input_ports.find(key);
    if (it == config.input_ports.end() || BT::TreeNode::isBlackboardPointer(it->second))
    {
        return std::nullopt;
    }
    return it->second;
}

/// Whether the XML named this port at all. Only meaningful for ports declared
/// without a default (see literal_port).
inline bool port_given(BT::NodeConfig const &config, std::string const &key)
{
    return config.input_ports.count(key) > 0;
}

/// A port validator's verdict: the complaint, or nothing when the value is
/// fine. Complaints name the port ("radius must be ...") so the message
/// stands on its own in a log.
using Problem = std::optional<std::string>;

/// Run a port's validator on its literal XML value, if it has one, and refuse
/// to build the tree when it objects. Nodes call the SAME validator on
/// {blackboard} values at run time, so each rule is written once.
template <typename T, typename Validator>
void reject_bad_literal(std::string const &name, BT::NodeConfig const &config, std::string const &key,
                        Validator const &validator)
{
    if (auto const literal = literal_port(config, key))
    {
        if (Problem const problem = validator(BT::convertFromString<T>(*literal)))
        {
            throw BT::RuntimeError(name, ": ", *problem, ", not \"", *literal, "\"");
        }
    }
}

/// Read an unsigned millisecond port (RosTimeout's msec, RosDelay's
/// delay_msec) and arm `budget` with it from `now_ns`. Returns the port's
/// error instead when it cannot be read, leaving the budget unarmed.
inline Problem arm_from_msec_port(BT::TreeNode const &node, std::string const &key, ros_time_budget::Budget &budget,
                                  std::int64_t now_ns)
{
    auto const msec = node.getInput<unsigned>(key);
    if (!msec)
    {
        return msec.error();
    }
    budget.arm(now_ns, ros_time_budget::clamp_msec(*msec));
    return std::nullopt;
}

}  // namespace prop_mission_planner
