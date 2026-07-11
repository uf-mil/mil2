#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Reads the latest classification published by the coin_flip classifier node
// (subjugator_vision, topic /coin_flip/direction) and, once the same side has
// persisted for several consecutive frames, writes it to the `wall_side` output
// port ("left"/"right"/"front"/"wall"). A mission can then Switch on wall_side
// to pick a move. "other"/empty is ignored (treated as undecided).
class DetectWallDirection : public BT::StatefulActionNode
{
  public:
    DetectWallDirection(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    std::string stable_side_;
    int stable_count_{ 0 };
    int need_frames_{ 3 };
};
