#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"
#include "detection_gate.hpp"

// Confirms a grasp by comparing the target's current apparent size to the baseline
// recorded before the lift (RecordTargetScale). A held object stays the same distance
// from the down camera, so its bbox area is roughly unchanged; a missed object stays on
// the table and shrinks as the sub lifts away. SUCCESS = grabbed, FAILURE = missed
// (which retries the grasp cycle). Degrades to SUCCESS (open-loop) when there is no
// baseline, e.g. before the down-cam model lands.
//
// Judges over MULTIPLE fresh post-start frames (was: one cached frame at tick
// time, which could be a PRE-LIFT frame where a missed object still looks
// full-size -> false GRABBED -> S5 transports an empty gripper):
//   - held  (present, ratio >= keep_ratio) x confirm_frames consecutive -> SUCCESS
//   - missed(present, ratio <  keep_ratio) x confirm_frames consecutive -> FAILURE
//   - absent x miss_frames consecutive -> absent_is_grabbed policy (ambiguous:
//     a held object can be occluded/too close; a missed one can drift off-frame)
//   - no verdict within timeout_msec (unstable readings / dead stream) ->
//     absent_is_grabbed policy, logged
class ConfirmGraspByScale : public BT::StatefulActionNode
{
  public:
    ConfirmGraspByScale(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label", "Target class to verify"),
                 BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
                 BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
                 BT::InputPort<double>("baseline_area", "Pre-lift bbox area from RecordTargetScale"),
                 BT::InputPort<double>("keep_ratio", 0.5, "Grabbed if current/baseline bbox area >= this"),
                 BT::InputPort<bool>("absent_is_grabbed", true,
                                     "If the target stays undetected after the lift, treat as grabbed (held object "
                                     "occluded/too close) rather than missed"),
                 // "3 to win": a verdict that costs the object's points (false
                 // GRABBED) or a full grasp retry (false MISSED) needs three
                 // consecutive fresh frames agreeing, ~0.2 s at pool rates.
                 BT::InputPort<int>("confirm_frames", 3,
                                    "Consecutive fresh frames agreeing (held or missed) required for a verdict"),
                 BT::InputPort<int>("miss_frames", 5,
                                    "Consecutive fresh frames without the target before the absent policy applies"),
                 BT::InputPort<int>("timeout_msec", 6000,
                                    "No stable verdict within this bound -> absent_is_grabbed policy"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override
    {
    }

  private:
    // RUNNING while inside the time bound; past it, decides from whichever
    // evidence streak leads (missed > held -> FAILURE, held > missed ->
    // SUCCESS), falling back to the absent_is_grabbed policy only on a tie.
    BT::NodeStatus timed_out_fallback(std::string const& label, bool absent_is_grabbed, int timeout_msec);

    std::shared_ptr<Context> ctx_;
    detection_gate::MissGate gate_;  // freshness + absent streak (gate_.misses)
    int held_streak_{ 0 };           // consecutive fresh frames: present & ratio >= keep
    int missed_streak_{ 0 };         // consecutive fresh frames: present & ratio < keep
    rclcpp::Time start_time_;
};
