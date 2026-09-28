#include "prop_mission_planner/ros_delay.hpp"

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
    if (!ctx_ && !(ctx_ = context_or_log(*this)))
    {
        return BT::NodeStatus::FAILURE;
    }
    if (!budget_.armed)
    {
        if (Problem const problem = arm_from_msec_port(*this, "delay_msec", budget_, ctx_->node->now().nanoseconds()))
        {
            RCLCPP_ERROR(ctx_->logger(), "%s: %s", name().c_str(), problem->c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
    setStatus(BT::NodeStatus::RUNNING);

    if (!budget_.expired(ctx_->node->now().nanoseconds()))
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
