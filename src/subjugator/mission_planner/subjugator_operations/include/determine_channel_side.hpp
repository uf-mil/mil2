#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

class DetermineChannelSide : public BT::StatefulActionNode
{
  public:
    DetermineChannelSide(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
    int right_count_{ 0 }, left_count_{ 0 };
    int need_frames_{ 3 };
    double min_conf_{ 0.30 };
};
