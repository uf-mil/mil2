#pragma once

#include <filesystem>
#include <iostream>
#include <optional>

#include <boost/asio.hpp>
#include <boost/dll.hpp>
#include <boost/process.hpp>

#include "mil_preflight/common.h"

namespace mil_preflight
{

class Frontend
{
  public:
    Frontend(int argc, char* argv[]);
    ~Frontend();

    void runJob(std::shared_ptr<Job> job);
    void runJobAsync(std::shared_ptr<Job> job);
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

    void runTest(std::shared_ptr<Test> test);
    void runAction(std::shared_ptr<Action> action);
};

}  // namespace mil_preflight
