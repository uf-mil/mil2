#pragma once

#include <behaviortree_cpp/behavior_tree.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "context.hpp"

/**
 * NavChannelControl
 *
 * Vision-based alignment controller for the navigation-channel gate. Each tick
 * it reads the latest YOLO detections, selects the nearest valid red/white gate
 * (or a single pole as a fallback), and emits ONE yaw OR strafe correction for
 * a downstream RelativeMove to execute.
 *
 * Return convention (drives a KeepRunningUntilFailure wrapper):
 *   RUNNING  - only from onStart()
 *   SUCCESS  - a command was computed this tick; advance the Sequence
 *   FAILURE  - poles centred for hold_ticks ticks (aligned / passing through);
 *              stop the loop
 *
 * This node does NOT command forward (x) motion — it is a pure alignment stage.
 * Forward progress through the gate is the job of a separate downstream move.
 */
class NavChannelControl : public BT::StatefulActionNode
{
  public:
    NavChannelControl(std::string const& name, BT::NodeConfiguration const& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    // ---- math helpers --------------------------------------------------------

    // Clamp v into [lo, hi].
    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    // Sigmoid mapping bbox height (px) -> weight in (0, 1).
    //   weight = 0.5 at height_px == center_px
    //   smaller div_px => steeper transition
    static double sigmoid_height(double height_px, double center_px, double div_px)
    {
        if (div_px <= 1e-9)
            return (height_px >= center_px) ? 1.0 : 0.0;
        return 1.0 / (1.0 + std::exp(-(height_px - center_px) / div_px));
    }

    // Signed ramp on |err|:
    //   |err| <= lo        -> 0            (dead zone)
    //   lo < |err| < hi    -> linear 0..1
    //   |err| >= hi        -> 1            (saturated)
    // Result carries the sign of err, so the output is in [-1, +1].
    static double signed_ramp(double err, double lo, double hi)
    {
        double const a = std::abs(err);
        if (a <= lo)
            return 0.0;
        double t;
        if (hi <= lo)
            t = 1.0;  // degenerate band -> behave as a hard step above lo
        else
            t = std::min(1.0, (a - lo) / (hi - lo));
        return (err < 0.0) ? -t : t;
    }

    // ---- state ---------------------------------------------------------------

    std::shared_ptr<Context> ctx_;

    // Channel side actually in use: +1 RIGHT, -1 LEFT, 0 unresolved.
    int resolved_side_{ 0 };

    // Consecutive ticks the gate has been within tol_px (discounted error).
    int centered_streak_{ 0 };

    // Previous-tick smoothed commands for the EMA output filter.
    double smoothed_yaw_{ 0.0 };
    double smoothed_y_{ 0.0 };
};
