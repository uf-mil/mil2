#include "ros_timeout.hpp"

#include <rclcpp/rclcpp.hpp>

RosTimeout::RosTimeout(std::string const& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config)
{
}

BT::PortsList RosTimeout::providedPorts()
{
    return {
        // Same port name as the builtin <Timeout> so call sites swap 1:1.
        BT::InputPort<int>("msec", "Budget in ROS-time milliseconds; child is halted and FAILURE returned on expiry"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus RosTimeout::tick()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "RosTimeout: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    if (!budget_.armed)
    {
        int msec = 0;
        if (!getInput("msec", msec))
        {
            RCLCPP_ERROR(ctx_->logger(), "RosTimeout: missing required port [msec]");
            return BT::NodeStatus::FAILURE;
        }
        budget_.arm(ctx_->node->now().nanoseconds(), msec);
    }
    setStatus(BT::NodeStatus::RUNNING);

    // Expiry is checked before ticking the child so nothing runs past the
    // deadline, mirroring the builtin (whose timer halts the child directly).
    if (budget_.expired(ctx_->node->now().nanoseconds()))
    {
        RCLCPP_WARN(ctx_->logger(), "RosTimeout: '%s' exceeded its budget, halting it", child_node_->name().c_str());
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
