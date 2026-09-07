#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cstdint>
#include <memory>
#include <string>

#include "context.hpp"
#include "detection_gate.hpp"

// Closed-loop XY visual servo: nudges the sub in body-frame surge/sway until a
// YOLO target's bounding-box centroid is centered in the (down-cam) image, then
// returns SUCCESS. Depth/yaw are held — intended to run at a fixed hover depth.
//
// Unlike HoneBearing (one-shot yaw), this stays RUNNING across many small
// correction steps, waiting for each step to settle before re-evaluating.
// Returns FAILURE only after `miss_frames` consecutive fresh detection frames
// lack the target (transient flicker is tolerated) so a Fallback can degrade
// to dead-reckoning. Wrap it in an XML <Timeout> to bound the loop.
class CenterCamera : public BT::StatefulActionNode
{
  public:
    CenterCamera(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    // Settle state is owned privately (not read back from the shared
    // ctx_->last_goal, which any goal-publishing node may overwrite): we wait
    // for the commanded step to be *traversed* from where we issued it.
    bool settling_{ false };      // a commanded correction is still in flight
    double step_start_x_{ 0.0 };  // sub XY when the in-flight step was issued
    double step_start_y_{ 0.0 };
    double step_dist_{ 0.0 };  // horizontal length of the in-flight step (m)
    // Frame gate: freshness (never act twice on one frame) + lost-vs-flicker
    // miss counting. Owns the last-considered stamp.
    detection_gate::MissGate gate_;
    int in_tol_count_{ 0 };  // consecutive fresh centered frames
};
