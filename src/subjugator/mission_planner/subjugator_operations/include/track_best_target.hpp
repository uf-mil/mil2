#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

/*
 * TrackBestTarget
 *
 * Single-detection sibling of TrackBestPair.
 *
 * - Reads latest YOLO detections from Context.
 * - Looks for the largest detection (bbox area) of a single class label
 *   (default "torpedoTarget").
 * - Keeps a "best-ever" score across ticks; whenever a frame beats it,
 *   snapshots the vehicle orientation from odom and outputs it as a
 *   quaternion so the mission can later yaw back to the heading that had
 *   the best view of the target. Position is not saved — pair the outputs
 *   with PublishGoalPose keep_current_pos_abs="true" for a yaw-only move.
 *
 * Always returns RUNNING, meant to be halted by a Parallel or Timeout in the BT.
 */

class TrackBestTarget : public BT::StatefulActionNode
{
  public:
    TrackBestTarget(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;

    // memorize-best state
    double best_score_ = -1e18;
};
