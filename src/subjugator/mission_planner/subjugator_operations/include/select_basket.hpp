#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Decision-only node (S5): maps the AUV's role (from Context) to its drop-off
// basket marker class and writes it to `basket_label` for TransportAndPlace.
// No motion, no detections. Mirror of SelectTarget's role->decision pattern.
class SelectBasket : public BT::StatefulActionNode
{
  public:
    SelectBasket(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
};
