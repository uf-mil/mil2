#pragma once
#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

class CountWhenTicked : public BT::SyncActionNode
{
  public:
    CountWhenTicked(std::string const& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        count_++;
        std::cout << "CounterAction ticked! count = " << count_ << std::endl;
        return BT::NodeStatus::SKIPPED;
    }

  private:
    int count_ = 0;
};
