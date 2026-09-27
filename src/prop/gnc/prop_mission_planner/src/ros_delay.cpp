#include "prop_mission_planner/ros_delay.hpp"

#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/context.hpp"
#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

REGISTER(RosDelay)

RosDelay::RosDelay(std::string const &name, BT::NodeConfig const &config) : BT::DecoratorNode(name, config)
{
    if (!port_given(config, "delay_msec"))
    {
        throw BT::RuntimeError(name, ": delay_msec is required (ROS-time milliseconds)");
    }
}

BT::PortsList RosDelay::providedPorts()
{
    return {
        // Same port name AND type as the builtin <Delay>, so call sites swap
        // 1:1 and a negative literal is rejected when the tree is built.
        BT::InputPort<unsigned>("delay_msec", "Pause in ROS-time milliseconds before the child is ticked"),
    };
}

BT::NodeStatus RosDelay::tick()
{
    auto const ctx = context_of(*this);
    if (!budget_.armed)
    {
        auto const delay_msec = getInput<unsigned>("delay_msec");
        if (!delay_msec)
        {
            RCLCPP_ERROR(ctx->logger(), "%s: %s", name().c_str(), delay_msec.error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        // Budget::arm takes a signed millisecond count; clamp rather than
        // overflow it on an (implausible) multi-week delay.
        auto const clamped_msec =
            static_cast<int>(std::min<unsigned>(*delay_msec, static_cast<unsigned>(std::numeric_limits<int>::max())));
        budget_.arm(ctx->node->now().nanoseconds(), clamped_msec);
    }
    setStatus(BT::NodeStatus::RUNNING);

    if (!budget_.expired(ctx->node->now().nanoseconds()))
    {
        return BT::NodeStatus::RUNNING;
    }

    auto const status = child_node_->executeTick();
    if (status != BT::NodeStatus::RUNNING)
    {
        budget_.disarm();
        resetChild();
    }
    return status;
}

void RosDelay::halt()
{
    budget_.disarm();
    BT::DecoratorNode::halt();
}

}  // namespace prop_mission_planner
