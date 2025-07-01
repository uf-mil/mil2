#pragma once

#include <filesystem>
#include <iostream>
#include <optional>

#include <boost/asio.hpp>
#include <boost/dll.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "mil_preflight/common.h"

namespace mil_preflight
{

class Frontend;
class Action
{
  public:
    friend class Frontend;

    Action() {};
    ~Action() {};

    virtual std::string const& getName() const = 0;
    virtual std::vector<std::string> const& getParameters() const = 0;

  protected:
    virtual void onStart() = 0;
    virtual void onFinish(bool success, std::string&& summery) = 0;
    virtual std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) = 0;

    std::vector<std::string> stdouts;
    std::vector<std::string> stderrs;

  private:
};

class Test
{
  public:
    friend class Frontend;

    Test() {};
    ~Test() {};

    virtual std::string const& getName() const = 0;
    virtual std::string const& getPlugin() const = 0;

  protected:
    virtual std::optional<std::reference_wrapper<Action>> nextAction() = 0;
    virtual void onFinish() = 0;
};

class Job
{
  public:
    friend class Frontend;

    Job() {};
    ~Job() {};

  protected:
    virtual std::optional<std::reference_wrapper<Test>> nextTest() = 0;
    virtual void onFinish() = 0;
};

class Frontend
{
  public:
    Frontend(int argc, char* argv[]);
    ~Frontend();

    void runJob(Job& job);
    void runJobAsync(Job& job);
    std::vector<std::string> const& getArgs()
    {
        return args_;
    }

  private:
    std::vector<std::string> args_;

    boost::filesystem::path bin_path_ = boost::process::search_path("mil_preflight_backend");

    boost::asio::io_context work_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread work_thread_;

    boost::process::ipstream child_out_;
    boost::process::ipstream child_err_;
    boost::process::opstream child_in_;

    boost::process::child backend_;

    void runTest(Test& test);
    void runAction(Action& action);
};

class UIBase
{
  public:
    UIBase()
    {
    }
    virtual ~UIBase()
    {
    }

    virtual int run([[maybe_unused]] std::shared_ptr<Frontend> frontend)
    {
        std::cout << "Unrecognized ui type " << frontend->getArgs()[0] << std::endl;
        return 1;
    }
};

}  // namespace mil_preflight
