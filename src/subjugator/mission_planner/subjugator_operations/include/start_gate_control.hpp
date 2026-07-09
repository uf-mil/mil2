#pragma once

#include <behaviortree_cpp/behavior_tree.h>

#include <algorithm>
#include <memory>
#include <string>

#include "context.hpp"

/**
 * StartGateControl
 *
 * Minimal p-controller on the horizontal screen position of one YOLO target.
 * Each tick: err = target_x_norm - 0.5, then yaw_cmd = clamp(-kp * err,
 * +/-max_yaw_deg). Inside `threshold` the target counts as centered and no
 * yaw is commanded.
 *
 * Return convention (drives a KeepRunningUntilFailure wrapper):
 *   SUCCESS - yaw step written to yaw_cmd; execute it with RelativeMove
 *   FAILURE - centered for kHoldTicks consecutive ticks (aligned); stop loop
 */
class StartGateControl : public BT::StatefulActionNode
{
  public:
    StartGateControl(std::string const& name, BT::NodeConfiguration const& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    // Debounce: consecutive centered ticks required before declaring aligned.
    static constexpr int kHoldTicks = 3;

    std::shared_ptr<Context> ctx_;
    int centered_streak_{ 0 };
};
