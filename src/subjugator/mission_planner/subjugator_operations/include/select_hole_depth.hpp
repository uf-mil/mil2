#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

/*
 * SelectHoleDepth
 *
 * The torpedo board has four holes at fixed positions, each at its own
 * height; the two board configurations only swap which task labels the top
 * and bottom pair (survey&repair on top with search&rescue on bottom, or
 * vice versa).
 *
 * Watches the YOLO detections until at least one hole of each task is
 * visible, compares their image heights to learn which task is on top, then
 * outputs the hard-coded depth of the requested hole: top/bottom from the
 * configuration, large/small from the label suffix. Pair the output with
 * AbsDepth so depth precision comes from odometry, not YOLO.
 *
 * Returns RUNNING until both tasks have been seen; bound it with a Timeout.
 */
class SelectHoleDepth final : public BT::StatefulActionNode
{
  public:
    SelectHoleDepth(std::string const& name, BT::NodeConfiguration const& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
};
