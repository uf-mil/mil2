#pragma once

#include <behaviortree_cpp/tree_node.h>

#include <optional>
#include <string>

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

}  // namespace prop_mission_planner
