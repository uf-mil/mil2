#include "prop_mission_planner/ros_timeout.hpp"

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/context.hpp"
#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

REGISTER(RosTimeout)

RosTimeout::RosTimeout(std::string const &name, BT::NodeConfig const &config) : BT::DecoratorNode(name, config)
{
    if (!port_given(config, "msec"))
    {
        throw BT::RuntimeError(name, ": msec is required (ROS-time milliseconds)");
    }
}

BT::PortsList RosTimeout::providedPorts()
{
    return {
        // Same port name as the builtin <Timeout>, so call sites swap 1:1.
        BT::InputPort<int>("msec", "Budget in ROS-time milliseconds; the child is halted and FAILURE returned on "
                                   "expiry"),
    };
}

BT::NodeStatus RosTimeout::tick()
{
    auto const ctx = context_of(*this);
    if (!budget_.armed)
    {
        auto const msec = getInput<int>("msec");
        if (!msec)
        {
            RCLCPP_ERROR(ctx->logger(), "%s: %s", name().c_str(), msec.error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        budget_.arm(ctx->node->now().nanoseconds(), *msec);
    }
    setStatus(BT::NodeStatus::RUNNING);

    // Checked before ticking the child, so nothing runs past the deadline.
    if (budget_.expired(ctx->node->now().nanoseconds()))
    {
        RCLCPP_WARN(ctx->logger(), "%s: '%s' ran out of time, halting it", name().c_str(), child_node_->name().c_str());
        haltChild();
        budget_.disarm();
        return BT::NodeStatus::FAILURE;
    }

    auto const status = child_node_->executeTick();
    if (status != BT::NodeStatus::RUNNING)
    {
        budget_.disarm();
        resetChild();
    }
    return status;
}

void RosTimeout::halt()
{
    budget_.disarm();
    BT::DecoratorNode::halt();
}

}  // namespace prop_mission_planner
