#pragma once
#include <behaviortree_cpp/action_node.h>

#include "context.hpp"

#include <geometry_msgs/msg/pose.hpp>

// P-controller that yaws the submarine until the YOLO detection bounding-box
// center is horizontally centred in the camera image (X axis).
// Returns RUNNING while correcting, SUCCESS when |error_x| < yaw_tol_norm.
class AlignYaw : public BT::StatefulActionNode
{
  public:
    AlignYaw(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    bool waiting_for_goal_{ false };
    geometry_msgs::msg::Pose pending_goal_;
};
