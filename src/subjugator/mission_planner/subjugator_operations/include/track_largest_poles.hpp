#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

// Tracks red/white poles and memorizes the best gate orientation seen during a survey.
// - Keeps legacy outputs (largest red/white areas) for compatibility.
// - NEW: scores red/white pairs per *current frame*; when a better pair is found,
//        snapshots odom orientation to outputs best_q* and best_score.
class TrackLargestPoles : public BT::StatefulActionNode
{
  public:
    TrackLargestPoles(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;

    // legacy "best area so far" (for the old outputs)
    double best_red_area_ = 0.0;
    double best_white_area_ = 0.0;

    // memorize-best state (for the new outputs)
    double best_pair_score_ = -1e18;

    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(hi, v));
    }
};
