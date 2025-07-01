#pragma once

#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_preflight/common.h"

namespace mil_preflight
{
class PluginBase : public rclcpp::Node
{
  public:
    PluginBase() : rclcpp::Node("mil_preflight_node")
    {
    }

    virtual ~PluginBase()
    {
    }

    virtual std::pair<bool, std::string> runAction([[maybe_unused]] std::shared_ptr<Action> action)
    {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
        return { false, "Failed to load plugin" };
    }
};

}  // namespace mil_preflight
