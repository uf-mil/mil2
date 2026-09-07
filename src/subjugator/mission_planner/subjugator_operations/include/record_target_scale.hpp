#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Records the apparent (pixel-area) size of the target in the down camera just before
// the lift, to be compared afterward by ConfirmGraspByScale. Writes 0.0 when the target
// is not currently detected (e.g. the down-cam model has not landed yet), which the
// confirm leaf reads as "no baseline -> skip verification" so the cycle stays open-loop.
// Always returns SUCCESS: recording a baseline should never fail the grasp cycle.
class RecordTargetScale : public BT::SyncActionNode
{
  public:
    RecordTargetScale(std::string const& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label", "Target class to measure"),
                 BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
                 BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
                 BT::OutputPort<double>("baseline_area", "Measured bbox area in pixels (0 if not seen)"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
};
