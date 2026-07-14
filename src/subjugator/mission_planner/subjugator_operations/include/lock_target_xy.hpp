#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cstdint>
#include <memory>
#include <string>

#include "context.hpp"
#include "detection_gate.hpp"
#include "target_projection.hpp"

// World-frame target lock (design G1). Like CenterCamera it centers the sub over
// a down-cam YOLO target, but instead of nudging by the image error it casts the
// detection's pixel ray onto the known table plane to estimate the target's
// WORLD (x,y), then commands an ABSOLUTE goal there. The setpoint is a fixed
// world point rather than a running current+step, which removes the
// moving-setpoint windup that defeated a position-layer integrator, and folds
// the fixed camera mount offset into the geometry exactly. Depth and yaw held.
// SUCCESS when |ex|,|ey| < tol_norm for settle_ticks fresh frames; FAILURE after
// miss_frames consecutive fresh misses. Wrap in <Timeout>.
class LockTargetXY : public BT::StatefulActionNode
{
  public:
    LockTargetXY(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    detection_gate::MissGate gate_;
    int in_tol_count_{ 0 };          // consecutive fresh centered frames
    bool have_estimate_{ false };    // EMA seeded yet?
    target_projection::Vec2 est_{};  // EMA of target world XY
};
