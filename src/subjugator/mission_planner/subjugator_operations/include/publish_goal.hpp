#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "context.hpp"
#include "operations.hpp"

// Publishes a Pose to /goal_pose.
class PublishGoalPose final : public BT::SyncActionNode, public OperationBase
{
  public:
    PublishGoalPose(std::string const& name, const BT::NodeConfiguration& cfg)
      : BT::SyncActionNode(name, cfg), OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    static geometry_msgs::msg::Pose composeAbsoluteGoal_(geometry_msgs::msg::Pose const& current, double rx, double ry,
                                                         double rz, double qx, double qy, double qz, double qw,
                                                         bool relative);
};
