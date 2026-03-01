#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "context.hpp"

class HoneMidpoint : public BT::StatefulActionNode
{
  public:
    HoneMidpoint(std::string const& name, BT::NodeConfiguration const& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;

    int centered_streak_ = 0;
    bool detected_midpoint_ = false;
    double last_red_cx_ = 0.0;
    double last_red_cy_ = 0.0;
    double last_white_cx_ = 0.0;
    double last_white_cy_ = 0.0;

    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(hi, v));
    }
};
