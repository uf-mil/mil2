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
        action_ = test->nextAction();

        while (action_ != nullptr)
        {
            action_->onStart();
            auto&& [success, summery] = runAction(action_->getName(), action_->getParameters());
            action_->onFinish(success, std::move(summery));
            action_ = test->nextAction();
        }

        test->onFinish();
    }

    int askQuestion(std::string&& question, std::vector<std::string>&& options)
    {
        return action_->onQuestion(std::move(question), std::move(options)).get();
    }

    virtual std::pair<bool, std::string> runAction(std::string const& name,
                                                   std::vector<std::string> const& parameters) = 0;

  private:
    std::shared_ptr<Action> action_;
};

}  // namespace mil_preflight
