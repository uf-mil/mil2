#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Confirms a grasp by comparing the target's current apparent size to the baseline
// recorded before the lift (RecordTargetScale). A held object stays the same distance
// from the down camera, so its bbox area is roughly unchanged; a missed object stays on
// the table and shrinks as the sub lifts away. SUCCESS = grabbed, FAILURE = missed
// (which retries the grasp cycle). Degrades to SUCCESS (open-loop) when there is no
// baseline, e.g. before the down-cam model lands.
class ConfirmGraspByScale : public BT::ConditionNode
{
  public:
    ConfirmGraspByScale(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label", "Target class to verify"),
                 BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
                 BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
                 BT::InputPort<double>("baseline_area", "Pre-lift bbox area from RecordTargetScale"),
                 BT::InputPort<double>("keep_ratio", 0.5, "Grabbed if current/baseline bbox area >= this"),
                 BT::InputPort<bool>("absent_is_grabbed", true,
                                     "If the target is not detected after the lift, treat as grabbed (held object "
                                     "occluded/too close) rather than missed"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
};
