#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>

#include <memory>

#include "context.hpp"

// Tiny base to access Context + shared helpers.
class OperationBase
{
  protected:
    std::shared_ptr<Context> ctx_;

  public:
    explicit OperationBase(const BT::NodeConfiguration& cfg)
    {
        ctx_ = cfg.blackboard->get<std::shared_ptr<Context>>("ctx");
    }
    virtual ~OperationBase() = default;

    static BT::PortsList commonPorts()
    {
        // Add shared ports here if you want (e.g., frame_id, debug)
        return {};
    }
};
