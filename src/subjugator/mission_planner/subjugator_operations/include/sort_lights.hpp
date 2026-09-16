#pragma once

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <cmath>

#include "light.hpp"

// Sorts `lights` in place by distance from (x, y), nearest first.
class SortLights final : public BT::SyncActionNode
{
  public:
    SortLights(std::string const& name, BT::NodeConfig const& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::BidirectionalPort<Lights>("lights", "Lights to sort"),
            BT::InputPort<double>("x", "Origin odom x"),
            BT::InputPort<double>("y", "Origin odom y"),
        };
    }

  private:
    BT::NodeStatus tick() override
    {
        Lights lights;
        double x = 0.0, y = 0.0;
        getInput("lights", lights);
        getInput("x", x);
        getInput("y", y);

        auto dist = [&](Light const& l) { return std::hypot(l.x - x, l.y - y); };
        std::sort(lights.begin(), lights.end(), [&](Light const& a, Light const& b) { return dist(a) < dist(b); });
        setOutput("lights", lights);
        return BT::NodeStatus::SUCCESS;
    }
};
