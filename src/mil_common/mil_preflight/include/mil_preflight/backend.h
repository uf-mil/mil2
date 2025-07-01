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

    virtual void runTest(std::shared_ptr<Test> test)
    {
        std::shared_ptr<mil_preflight::Action> action = test->nextAction();

        while (action != nullptr)
        {
            action->onStart();
            action->onFinish(false, "Failed to load the plugin " + test->getPlugin());
            action = test->nextAction();
        }

        test->onFinish();
    }
};

class SimplePlugin : public PluginBase
{
  public:
    SimplePlugin()
    {
    }

    virtual ~SimplePlugin()
    {
    }

  protected:
    virtual void runTest(std::shared_ptr<Test> test) override
    {
        std::shared_ptr<mil_preflight::Action> action = test->nextAction();

        while (action != nullptr)
        {
            action->onStart();
            auto &&[success, summery] = runAction(action);
            action->onFinish(success, std::move(summery));
            action = test->nextAction();
        }

        test->onFinish();
    }

    virtual std::pair<bool, std::string> runAction(std::shared_ptr<Action> action) = 0;
};

class Backend
{
  public:
    using Creator = std::shared_ptr<mil_preflight::PluginBase>;

    Backend();
    ~Backend();

    void run(rclcpp::executors::SingleThreadedExecutor &exec);

  private:
    boost::function<Creator()> load(std::string const &plugin_name);
    std::unordered_map<std::string, boost::function<Creator()>> libraries_;
};

}  // namespace mil_preflight
