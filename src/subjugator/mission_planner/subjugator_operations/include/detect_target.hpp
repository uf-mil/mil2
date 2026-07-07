#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"
#include "detection_gate.hpp"

// Flicker-tolerant presence guard: SUCCESS after `confirm_frames` consecutive
// fresh detection frames containing `label` (default 1 — one FRESH sighting),
// FAILURE only after `miss_frames` consecutive fresh frames without it, or
// after `timeout_msec` with no verdict (e.g. dead stream).
//
// Was a synchronous ConditionNode judging the single cached frame at tick
// time — one glinted frame as the acquire_table spiral guard bought a
// spurious 60 s / 2 m search away from a table that was in view. A
// synchronous node ticked once cannot wait for a second frame, hence the
// conversion to a stateful action (XML call sites use the bare tag or
// <Action ID=...>).
class DetectTarget : public BT::StatefulActionNode
{
  public:
    DetectTarget(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label", "shark", "Target label (single)"),
                 BT::InputPort<double>("min_conf", 0.40, "Minimum confidence"),
                 BT::InputPort<std::string>("camera", "front", "Detection stream: 'front' or 'down'"),
                 // confirm=1: the freshness requirement alone already kills the
                 // stale-cached-frame false positive; both call sites tolerate
                 // a cheap false positive (hone / gate-approach verify next)
                 // but pay dearly for a false negative (60 s spiral).
                 BT::InputPort<int>("confirm_frames", 1,
                                    "Consecutive fresh frames with the label required for SUCCESS"),
                 BT::InputPort<int>("miss_frames", 5, "Consecutive fresh frames without the label before FAILURE"),
                 BT::InputPort<int>("timeout_msec", 5000, "Give up (FAILURE) if no verdict within this bound"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
    detection_gate::MissGate gate_;
    rclcpp::Time start_time_;
};
