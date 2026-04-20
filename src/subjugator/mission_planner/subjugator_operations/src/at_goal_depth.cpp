#include "at_goal_depth.hpp"

#include <cmath>
#include <iostream>

AtGoalDepth::AtGoalDepth(std::string const& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{
}

// void AtGoalDepth::depthCallback(const mil_msgs::msg::DepthStamped::SharedPtr msg) {
//     depth.store(msg->depth);
// }

BT::NodeStatus AtGoalDepth::tick()
{
    double target;
    double tolerance;
    getInput("target_depth", target);
    getInput("tolerance", tolerance);
    getInput("ctx", ctx);

    std::scoped_lock lk(ctx->odom_mx);
    auto odom = ctx->latest_odom;
    auto current_depth = odom->pose.pose.position.z;
    // auto current_depth = depth.load();

    if (std::isnan(current_depth))
    {
        return BT::NodeStatus::FAILURE;
    }

    double error = std::abs(current_depth - target);
    std::cout << "target: " << target << std::endl;
    std::cout << "current: " << current_depth << std::endl;
    std::cout << "error: " << error << std::endl;

    if (std::abs(error) < tolerance)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
