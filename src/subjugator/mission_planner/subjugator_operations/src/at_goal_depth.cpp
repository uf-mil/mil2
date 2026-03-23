#include "at_goal_depth.hpp"

#include <cmath>

BT::NodeStatus AtGoalDepth::tick()
{
    double target;
    double tolerance;
    getInput("target_depth", target);
    getInput("tolerance", tolerance);
    getInput("ctx", ctx_);

    // copied from at_goal_pose
    std::optional<nav_msgs::msg::Odometry> odom;
    {
        std::scoped_lock lk(ctx_->odom_mx);
        odom = ctx_->latest_odom;
    }

    if (!odom)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "AtGoalDepth: no odometry yet");
        return BT::NodeStatus::FAILURE;
    }

    auto current_depth = odom->pose.pose.position.z;

    double error = std::abs(current_depth - target);

    if (error < tolerance)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
