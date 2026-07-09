#pragma once
#include <behaviortree_cpp/action_node.h>

#include "context.hpp"

#include <geometry_msgs/msg/pose.hpp>

// P-controller that translates the submarine sideways (body-frame Y) until the
// YOLO detection bounding-box center is horizontally centered in the camera
// image. Drop-in alternative to AlignYaw for testing strafe vs. yaw alignment.
// Returns RUNNING while correcting, SUCCESS when |error_x| < strafe_tol_norm.
class AlignStrafe : public BT::StatefulActionNode
{
  public:
    AlignStrafe(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    bool waiting_for_goal_{ false };
    geometry_msgs::msg::Pose pending_goal_;
};
