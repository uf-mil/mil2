#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/maneuvers.hpp"

namespace prop_mission_planner
{

class ManeuverNode;

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
    /// The maneuver node currently driving, or nullptr. The lock and the
    /// motors are shared, so a second maneuver node refuses to start while
    /// one is claimed instead of silently re-aiming the first one's boat.
    ManeuverNode const *active_maneuver{ nullptr };

    rclcpp::Logger logger() const
    {
        return node->get_logger();
    }

    /// Stop everything that can command the motors: guidance gets an empty
    /// path and the spinner and reverser each send a zero command. Safe to call
    /// at any time, including when nothing is moving. Call it before
    /// rclcpp::shutdown(): after shutdown the stop messages cannot be
    /// published.
    void stop_motors();
};

/// Build a Context on `node`: declare the maneuver parameters, then create the
/// maneuvers' shared state. Call once per node: the maneuver parameters can
/// only be declared once.
std::shared_ptr<Context> make_context(rclcpp::Node::SharedPtr const &node);

/// The Context on the ROOT blackboard ("@ctx"), so nodes inside subtrees find
/// it without remapping. Throws BT::RuntimeError when it is missing, which
/// only happens if a tree was created without one, and BT's own conversion
/// error if "ctx" holds a different type.
std::shared_ptr<Context> context_of(BT::TreeNode const &tree_node);

/// The factory every node registers into. A function-local static, so
/// registration from other files' static constructors never runs before it
/// exists (the sub's global-variable version depends on link order).
BT::BehaviorTreeFactory &factory();

}  // namespace prop_mission_planner

/// Register a node type under its class name. Use inside namespace
/// prop_mission_planner, in the node's .cpp file, AFTER anything its ports
/// use: this is an ordinary static initialiser, so it runs in declaration
/// order within the file (a constructor-attribute function would run before
/// the file's own static objects exist).
#define REGISTER(name)                                                                                                 \
    namespace                                                                                                          \
    {                                                                                                                  \
    [[maybe_unused]] bool const registered_##name =                                                                    \
        (::prop_mission_planner::factory().registerNodeType<name>(#name), true);                                       \
    }
