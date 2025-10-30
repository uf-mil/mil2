#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

class DetectTarget : public BT::ConditionNode
{
  public:
    DetectTarget(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label", "shark", "Target label (single)"),
                 BT::InputPort<double>("min_conf", 0.40, "Minimum confidence"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
};
