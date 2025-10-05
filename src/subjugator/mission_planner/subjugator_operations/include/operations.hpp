#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "context.hpp"

// Tiny base to access Context + shared helpers.
class OperationBase
{
  public:
    explicit OperationBase(const BT::NodeConfiguration &)
    {
    }
    virtual ~OperationBase() = default;

    static BT::PortsList commonPorts()
    {
        return {};
    }

  protected:
    std::shared_ptr<Context> ctx_;
};
