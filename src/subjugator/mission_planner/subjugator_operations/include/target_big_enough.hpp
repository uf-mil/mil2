#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Returns SUCCESS when the largest detection matching `label` has a bounding box
// that meets or exceeds the configured pixel size thresholds for `consecutive_frames`
// consecutive frames; otherwise FAILURE. Apparent size grows as we get closer, so
// this works as an "approach until the target is big enough (close enough)" gate,
// e.g. creeping toward the torpedo board (label "torpedoTarget").
class TargetBigEnough : public BT::ConditionNode
{
  public:
    TargetBigEnough(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
    int ok_streak_{ 0 };  // consecutive good frames seen so far
};
