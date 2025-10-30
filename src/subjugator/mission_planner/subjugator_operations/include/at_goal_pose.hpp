#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "context.hpp"
#include "operations.hpp"

// Checks whether current odom is within tolerance of a given goal
class AtGoalPose final : public BT::ConditionNode, public OperationBase
{
  public:
    AtGoalPose(std::string const& name, const BT::NodeConfiguration& cfg)
      : BT::ConditionNode(name, cfg), OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    static double quatAngularErrorDeg_(double qx, double qy, double qz, double qw, double rx, double ry, double rz,
                                       double rw);
};
