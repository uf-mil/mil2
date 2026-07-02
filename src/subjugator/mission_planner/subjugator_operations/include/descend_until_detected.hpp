#pragma once
#include <behaviortree_cpp/action_node.h>

#include "context.hpp"

#include <geometry_msgs/msg/pose.hpp>

// Descends in fixed Z steps until the down-cam YOLO reports `label`, or a
// step/time bound is hit. SUCCESS when detected; FAILURE at max_steps/timeout.
// Step-and-wait goal publishing mirrors AlignDepth.
class DescendUntilDetected : public BT::StatefulActionNode
{
  public:
    DescendUntilDetected(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    bool waiting_for_goal_{ false };
    geometry_msgs::msg::Pose pending_goal_;
    int steps_taken_{ 0 };
    rclcpp::Time start_time_;
    bool label_seen(std::string const& label, std::string const& camera, double min_conf);
};
