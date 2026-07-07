#pragma once
#include <behaviortree_cpp/action_node.h>

#include "context.hpp"
#include "detection_gate.hpp"

#include <geometry_msgs/msg/pose.hpp>

// Descends in fixed Z steps until the down-cam YOLO reports `label` on
// confirm_frames consecutive FRESH detection frames (a single spurious frame
// must not end the descent too high), or a step/time bound is hit.
// SUCCESS when confirmed; FAILURE at max_steps/timeout. Step-and-wait goal
// publishing mirrors AlignDepth.
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
    // Freshness + consecutive-hit confirmation (seeded at onStart so a cached
    // pre-start frame can't instantly satisfy the detection).
    detection_gate::MissGate gate_;
    // No stepping until the first post-start frame has been evaluated (so a
    // target already in view confirms at the start depth, with zero steps).
    bool seen_fresh_{ false };
};
