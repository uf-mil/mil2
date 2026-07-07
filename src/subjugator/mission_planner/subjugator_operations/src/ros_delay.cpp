#include "ros_delay.hpp"

#include <rclcpp/rclcpp.hpp>

RosDelay::RosDelay(std::string const& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config)
{
}

BT::PortsList RosDelay::providedPorts()
{
    return {
        // Same port name as the builtin <Delay> so call sites swap 1:1.
        BT::InputPort<int>("delay_msec", "Pause in ROS-time milliseconds before the child is ticked"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus RosDelay::tick()
{
    if (!require_ctx(*this, ctx_, "RosDelay"))
    {
        return BT::NodeStatus::FAILURE;
    }
    if (!budget_.armed)
    {
        int delay_msec = 0;
        if (!getInput("delay_msec", delay_msec))
        {
            RCLCPP_ERROR(ctx_->logger(), "RosDelay: missing required port [delay_msec]");
            return BT::NodeStatus::FAILURE;
        }
        budget_.arm(ctx_->node->now().nanoseconds(), delay_msec);
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
