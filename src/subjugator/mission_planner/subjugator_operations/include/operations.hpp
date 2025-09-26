#pragma once
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

class Operation
{
  public:
    enum class opStatus : uint8_t
    {
        Running,
        Succeeded,
        Failed,
        Canceled,
        Idle,
    }

    Operation();

    virtual ~Operation() = default;
}
