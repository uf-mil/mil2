#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

/*
 * TrackBestPair
 *
 * - Reads latest YOLO detections from Context.
 * - Selects top-K red and white pole detections (by area * confidence).
 * - Scores pairs and keeps a "best-ever" score across ticks.
 * - When the best-ever score is improved, snapshots and outputs the vehicle orientation.
 *
 * Always returns RUNNING, meant to be bounded by a timeout in the BT
 */

class TrackBestPair : public BT::StatefulActionNode
{
  public:
    TrackBestPair(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;

    // memorize-best state
    double best_pair_score_ = -1e18;
};
