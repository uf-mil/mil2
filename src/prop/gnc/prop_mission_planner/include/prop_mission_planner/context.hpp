#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/maneuvers.hpp"

namespace prop_mission_planner
{

/// Everything a mission's nodes share. One per run, on the root blackboard
/// under "ctx".
struct Context
{
    rclcpp::Node::SharedPtr node;
    /// The maneuver settings, declared on `node` so maneuvers.yaml (keyed /**)
    /// applies unchanged when passed as --params-file.
    std::unique_ptr<prop_maneuvers::Constants> settings;
    /// Odometry, Driver, Spinner, Reverser and TargetLock. Only one maneuver
    /// node runs at a time, so they all share this one.
    std::unique_ptr<prop_maneuvers::Context> maneuvers;

    rclcpp::Logger logger() const
    {
        return node->get_logger();
    }

    /// Stop everything that can command the motors: guidance gets an empty
    /// path and the spinner and reverser each send a zero command. Safe to call
    /// at any time, including when nothing is moving.
    void stop_motors();
};

/// Build a Context on `node`: declare the maneuver parameters, then create the
/// maneuvers' shared state.
std::shared_ptr<Context> make_context(rclcpp::Node::SharedPtr const &node);

/// The Context on the ROOT blackboard ("@ctx"), so nodes inside subtrees find
/// it without remapping. Throws BT::RuntimeError when it is missing, which
/// only happens if a tree was created without one.
std::shared_ptr<Context> context_of(BT::TreeNode const &tree_node);

/// The factory every node registers into. A function-local static, so
/// registration from other files' static constructors never runs before it
/// exists (the sub's global-variable version depends on link order).
BT::BehaviorTreeFactory &factory();

}  // namespace prop_mission_planner

/// Register a node type under its class name. Use inside namespace
/// prop_mission_planner, in the node's .cpp file.
#define REGISTER(name)                                                                                                 \
    extern "C" __attribute__((constructor)) void register_##name()                                                     \
    {                                                                                                                  \
        ::prop_mission_planner::factory().registerNodeType<name>(#name);                                               \
    }
