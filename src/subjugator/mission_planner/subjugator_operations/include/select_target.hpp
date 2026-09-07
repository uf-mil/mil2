#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <cstdint>
#include <memory>
#include <string>

#include "context.hpp"

// Decision-only down-cam node: selects which role-target object to grab and
// writes its class to the `target_label` output port for S4. No motion.
class SelectTarget : public BT::StatefulActionNode
{
  public:
    SelectTarget(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
    std::string leader_;                // current front-runner class
    int votes_{ 0 };                    // consecutive distinct frames leader has won
    std::int64_t last_stamp_ns_{ -1 };  // last detection frame already counted
};
