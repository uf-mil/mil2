#pragma once

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "context.hpp"

// NavChannelControl
//
// Per-tick controller for navigating a red/white channel gate. Picks the
// largest red and white poles in the current frame, weights each pole's
// position error by a size-based sigmoid (closer pole -> larger weight), and
// emits EITHER a yaw step OR a strafe step (never both in the same tick).
//
// right_channel input:
//   +1 -> RIGHT channel  (red expected left-of-center, white right-of-center)
//   -1 -> LEFT channel   (red expected right-of-center, white left-of-center)
//    0 -> AUTO: latches the side from the first frame containing both colors.
//
// Output `resolved_channel` reports the side actually used and can be wired
// back to the same blackboard variable as the input to persist the choice.
//
// SUCCESS once every visible pole's pixel error stays within `tol_px` for
// `hold_ticks` consecutive ticks.
class NavChannelControl : public BT::StatefulActionNode
{
  public:
    NavChannelControl(std::string const& name, BT::NodeConfiguration const& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;

    int centered_streak_ = 0;
    int resolved_side_ = 0;  // +1 RIGHT, -1 LEFT, 0 unknown

    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    // Sigmoid on pole height. Larger height (closer pole) => weight -> 1.
    static double sigmoid_height(double height_px, double center_px, double div_px)
    {
        if (div_px <= 1e-9)
            return (height_px >= center_px) ? 1.0 : 0.0;
        return 1.0 / (1.0 + std::exp(-(height_px - center_px) / div_px));
    }

    // Signed linear ramp: 0 at |e|<=lo, +/-1 at |e|>=hi, sign matches e.
    static double signed_ramp(double e, double lo, double hi)
    {
        double const a = std::abs(e);
        if (a <= lo)
            return 0.0;
        if (hi <= lo)
            return (e > 0.0) ? 1.0 : -1.0;
        double const r = std::min(1.0, (a - lo) / (hi - lo));
        return (e > 0.0 ? 1.0 : -1.0) * r;
    }
};
