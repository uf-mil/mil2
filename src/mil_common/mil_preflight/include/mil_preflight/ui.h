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

class UIBase;
class Action
{
  public:
    friend class UIBase;
    struct Report
    {
        bool success;
        std::string summery;
        std::vector<std::string> stdouts;
        std::vector<std::string> stderrs;

        Report() : success(false)
        {
        }
        ~Report() = default;
        Report(Report const& report) = default;
        Report(Report&& report)
        {
            success = report.success;
            report.success = false;
            summery = std::move(report.summery);
            stdouts = std::move(report.stdouts);
            stderrs = std::move(report.stderrs);
        }

        Report& operator=(Report const& report)
        {
            success = report.success;
            summery = report.summery;
            stdouts = report.stdouts;
            stderrs = report.stderrs;

            return *this;
        }

        Report& operator=(Report&& report)
        {
            success = report.success;
            report.success = false;
            summery = std::move(report.summery);
            stdouts = std::move(report.stdouts);
            stderrs = std::move(report.stderrs);

            return *this;
        }
    };

    Action() {};
    ~Action() {};

    virtual std::string const& getName() const = 0;
    virtual std::vector<std::string> const& getParameters() const = 0;

  protected:
    virtual void onStart() = 0;
    virtual void onFinish(Report&& report) = 0;

  private:
};

class Test
{
  public:
    friend class UIBase;

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
    friend class UIBase;

    Job() {};
    ~Job() {};

  protected:
    virtual std::optional<std::reference_wrapper<Test>> nextTest() = 0;
    virtual void onFinish() = 0;
};

class UIBase
{
  public:
    UIBase()
      : work_guard(boost::asio::make_work_guard(work_context))
      , work_thread([this] { work_context.run(); })
      , backend(boost::process::child(bin_path.string(),
                                      boost::process::std_in<child_in, boost::process::std_out> child_out,
                                      boost::process::std_err > child_err))
    {
    }

    virtual ~UIBase()
    {
        work_guard.reset();
        work_context.stop();
        work_thread.join();

        if (backend.running())
            child_in << EOT << std::endl;

        backend.join();
    }

    virtual int spin()
    {
        return -1;
    }

    virtual bool initialize([[maybe_unused]] int argc, char* argv[])
    {
        std::cout << "Unrecognized ui type " << argv[0] << std::endl;
        return false;
    }

    void runJob(Job& job)
    {
        std::optional<std::reference_wrapper<Test>> testOptional = job.nextTest();

        while (testOptional.has_value())
        {
            Test& test = testOptional.value();
            runTest(test);

            testOptional = job.nextTest();
        }

        job.onFinish();
    }

    void runJobAsync(Job& job)
    {
        work_context.post([&] { runJob(job); });
    }

  protected:
    virtual std::shared_future<int> onQuestion([[maybe_unused]] std::string&& question,
                                               [[maybe_unused]] std::vector<std::string>&& options)
    {
        std::promise<int> question_promise;
        question_promise.set_value(-1);
        return question_promise.get_future().share();
    }

  private:
    boost::filesystem::path bin_path = boost::process::search_path("mil_preflight_backend");

    boost::asio::io_context work_context;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard;
    std::thread work_thread;

    boost::process::ipstream child_out;
    boost::process::ipstream child_err;
    boost::process::opstream child_in;

    boost::process::child backend;

    void runTest(Test& test)
    {
        std::optional<std::reference_wrapper<Action>> actionOptional = test.nextAction();

        child_in << test.getPlugin() << std::endl;

        while (actionOptional.has_value())
        {
            Action& action = actionOptional.value();
            runAction(action);

            actionOptional = test.nextAction();
        }

        if (!child_in.fail())
            child_in << EOT << std::endl;

        test.onFinish();
    }

    void runAction(Action& action)
    {
        action.onStart();

        Action::Report actionReport;

        enum class State
        {
            START,
            STDOUT,
            SUMMERY,
            QUESTION,
            OPTIONS,
        } state = State::START;

        std::string line;

        std::string question;
        std::vector<std::string> options;

        while (true)
        {
            if (state == State::START)
            {
                try
                {
                    child_in << action.getName() << std::endl;
                    for (std::string const& parameter : action.getParameters())
                    {
                        child_in << parameter << std::endl;
                    }
                    child_in << GS << std::endl;
                }
                catch (std::exception const& e)
                {
                    actionReport.summery = "Broken pipe: " + std::string(e.what());
                    break;
                }
                state = State::STDOUT;
            }
            else if (state == State::STDOUT)
            {
                if (!std::getline(child_out, line))
                {
                    actionReport.summery = "Broken pipe";
                    break;
                }

                if (line[0] == ACK)
                {
                    state = State::SUMMERY;
                    actionReport.success = true;
                }
                else if (line[0] == NCK)
                {
                    state = State::SUMMERY;
                }
                else if (line[0] == BEL)
                {
                    state = State::QUESTION;
                }
                else
                {
                    actionReport.stdouts.push_back(std::move(line));
                }
            }
            else if (state == State::QUESTION)
            {
                if (!std::getline(child_out, question, GS))
                {
                    actionReport.summery = "Broken pipe";
                    break;
                }

                state = State::OPTIONS;
            }
            else if (state == State::OPTIONS)
            {
                if (!std::getline(child_out, line, GS))
                {
                    actionReport.summery = "Broken pipe";
                    break;
                }

                if (line[0] == EOT)
                {
                    std::shared_future<int> feedback = onQuestion(std::move(question), std::move(options));
                    try
                    {
                        child_in << feedback.get() << std::endl;
                        options.clear();
                    }
                    catch (std::exception const& e)
                    {
                        actionReport.summery = "Broken pipe: " + std::string(e.what());
                        break;
                    }
                    state = State::STDOUT;
                }
                else
                {
                    options.push_back(std::move(line));
                }
            }
            else if (state == State::SUMMERY)
            {
                if (!std::getline(child_out, actionReport.summery))
                {
                    break;
                }

                while (std::getline(child_err, line))
                {
                    if (line[0] == EOT)
                        break;
                    actionReport.stderrs.push_back(std::move(line));
                }

                break;
            }
        }

        action.onFinish(std::move(actionReport));
    }
};

}  // namespace mil_preflight
