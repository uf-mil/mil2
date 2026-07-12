#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

// Condition: SUCCESS when the best detection of `label` has its bounding-box
// center within `tol_norm` of the image center on BOTH axes (normalised so 1.0 =
// image edge). Used as the "fire only when we're facing it / it's close enough to
// the center of view" gate before launching a torpedo. FAILURE if the target is
// not detected, not centered, or the image size is unknown.
class TargetCentered : public BT::ConditionNode
{
  public:
    TargetCentered(std::string const& name, const BT::NodeConfiguration& cfg) : BT::ConditionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<Context> ctx_;
};
