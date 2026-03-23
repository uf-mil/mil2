#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include "context.hpp"
#include "operations.hpp"

// checks whether the sub has reached a goal depth
class AtGoalDepth : public BT::ConditionNode
{
  public:
    AtGoalDepth(std::string const& name, const BT::NodeConfiguration& cfg)
      : BT::ConditionNode(name, cfg)  //, OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports;
        ports.insert(BT::InputPort<double>("target_depth"));
        ports.insert(BT::InputPort<double>("tolerance"));
        ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx"));
        return ports;
    }
    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
    double curr_depth;
};
