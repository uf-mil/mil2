#pragma once
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include "context.hpp"
#include "operations.hpp"

// GetPathMarkerHeading
//
// A StatefulActionNode that waits until path_marker_heading.py
// publishes a quaternion on /path_marker/heading, then writes
// the four quaternion components to output ports so the XML
// tree can feed them straight into PublishGoalPose.
//
// XML usage:
//   <Action ID="GetPathMarkerHeading"
//           timeout_sec="20.0"
//           ctx="{ctx}"
//           qx="{pm_qx}" qy="{pm_qy}" qz="{pm_qz}" qw="{pm_qw}"/>

class GetPathMarkerHeading : public BT::StatefulActionNode, public OperationBase
{
  public:
    GetPathMarkerHeading(std::string const& name, BT::NodeConfiguration const& cfg)
      : BT::StatefulActionNode(name, cfg), OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            // Input
            BT::InputPort<std::shared_ptr<Context>>("ctx"),
            BT::InputPort<double>("timeout_sec", 20.0, "How long to wait for a detection"),

            // Outputs — feed these into PublishGoalPose
            BT::OutputPort<double>("qx"),
            BT::OutputPort<double>("qy"),
            BT::OutputPort<double>("qz"),
            BT::OutputPort<double>("qw"),
        };
    }

    // Called once when the node first becomes active
    BT::NodeStatus onStart() override;

    // Called every tick while RUNNING
    BT::NodeStatus onRunning() override;

    // Called if the tree halts this node early
    void onHalted() override
    {
    }

  private:
    rclcpp::Time start_time_;
    double timeout_sec_{ 20.0 };
};
