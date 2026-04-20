#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include "rclcpp/rclcpp.hpp"

#include "context.hpp"
#include "mil_msgs/msg/depth_stamped.hpp"
#include "operations.hpp"

// checks whether the sub has reached a goal depth
class AtGoalDepth : public BT::ConditionNode
{
  public:
    AtGoalDepth(std::string const& name, const BT::NodeConfiguration& cfg);

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports;
        ports.insert(BT::InputPort<double>("target_depth"));
        ports.insert(BT::InputPort<double>("tolerance"));
        ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx"));
        return ports;
    }
    BT::NodeStatus tick() override;

  private:
    // void depthCallback(const mil_msgs::msg::DepthStamped::SharedPtr msg);
    // rclcpp::Node::SharedPtr node;
    // rclcpp::Subscription<mil_msgs::msg::DepthStamped>::SharedPtr subscription;
    // std::atomic<double> depth{std::numeric_limits<double>::quiet_NaN()};

    std::shared_ptr<Context> ctx;
};
