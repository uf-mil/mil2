#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

class HoneBearing : public BT::StatefulActionNode
{
  public:
    HoneBearing(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    bool published_{ false };
    bool warned_empty_labels_{ false };  // one-shot: 'labels' set but parsed to no classes
};
