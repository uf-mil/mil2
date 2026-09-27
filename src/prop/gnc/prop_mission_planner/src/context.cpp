#include "prop_mission_planner/context.hpp"

namespace prop_mission_planner
{

void Context::stop_motors()
{
    // Guidance first: on an empty path its timer stops publishing, so the zero
    // commands below are not immediately overwritten by a drive command.
    maneuvers->driver.release();
    maneuvers->spinner.stop();
    maneuvers->reverser.stop();
}

std::shared_ptr<Context> make_context(rclcpp::Node::SharedPtr const &node)
{
    auto ctx = std::make_shared<Context>();
    ctx->node = node;
    ctx->settings = std::make_unique<prop_maneuvers::Constants>(node.get());
    ctx->maneuvers = std::make_unique<prop_maneuvers::Context>(node.get(), *ctx->settings);
    return ctx;
}

std::shared_ptr<Context> context_of(BT::TreeNode const &tree_node)
{
    auto const &blackboard = tree_node.config().blackboard;
    std::shared_ptr<Context> ctx;
    if (!blackboard || !blackboard->get("@ctx", ctx) || !ctx)
    {
        throw BT::RuntimeError(tree_node.name(), ": no Context on the root blackboard under \"ctx\"");
    }
    return ctx;
}

BT::BehaviorTreeFactory &factory()
{
    static BT::BehaviorTreeFactory instance;
    return instance;
}

}  // namespace prop_mission_planner
