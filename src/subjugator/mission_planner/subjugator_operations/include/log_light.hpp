#pragma once

#include <behaviortree_cpp/action_node.h>

#include "context.hpp"
#include "light.hpp"

// Appends the last goal pose (where the sub was just sent, i.e. over the light) to `lights`.
class LogLight final : public BT::SyncActionNode
{
  public:
    LogLight(std::string const& name, BT::NodeConfig const& cfg)
      : BT::SyncActionNode(name, cfg), ctx_(requireContext(*this))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("color", "Colour of the light"),
            BT::BidirectionalPort<Lights>("lights", "Lights logged so far"),
            BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;

    BT::NodeStatus tick() override
    {
        std::string color;
        Lights lights;
        getInput("color", color);
        getInput("lights", lights);

        auto const pos = ctx_->lastGoalOrOdom().position;
        lights.push_back({ color, pos.x, pos.y });
        setOutput("lights", lights);

        RCLCPP_INFO(ctx_->logger(), "Logged %s light #%zu at (%.2f, %.2f)", color.c_str(), lights.size(), pos.x, pos.y);
        return BT::NodeStatus::SUCCESS;
    }
};
