#pragma once
#include <behaviortree_cpp/action_node.h>

#include "context.hpp"

#include <geometry_msgs/msg/pose.hpp>

// Orbits ("arches") the submarine sideways around the torpedo board so we end up
// facing it head-on, using the board's 4 corner keypoints from the YOLO pose model.
//
// Keypoint ids (1-based, from yolo_node): 1 = top-left, 2 = top-right,
// 3 = bottom-left, 4 = bottom-right of the board.
//
// We compare the apparent length of the LEFT vertical edge (kp1->kp3) with the
// RIGHT vertical edge (kp2->kp4). Viewing a planar board obliquely makes the
// nearer edge look longer, so an imbalance means we are off to one side:
//   left edge longer  -> board's left side is closer  -> arch RIGHT (default)
//   right edge longer -> board's right side is closer -> arch LEFT  (default)
// (Flip dir_sign to invert if the physical convention comes out reversed.)
//
// Each tick that needs correcting commands ONE small orbit step (sway + yaw at a
// tunable radius) and waits for arrival, then re-evaluates. Returns SUCCESS once
// the edges are balanced within deadband_frac, or once max_steps is reached so we
// never arch indefinitely. Bound it with a Timeout in the tree as well.
class BoardArchStep : public BT::StatefulActionNode
{
  public:
    BoardArchStep(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    bool waiting_for_goal_{ false };
    geometry_msgs::msg::Pose pending_goal_;
    int steps_done_{ 0 };
};
