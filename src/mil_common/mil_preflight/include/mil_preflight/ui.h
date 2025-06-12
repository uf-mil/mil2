#pragma once

#include <filesystem>
#include <iostream>
#include <optional>

#include <boost/asio.hpp>
#include <boost/dll.hpp>
#include <boost/function.hpp>
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
    virtual void onFinish(Report const& report) = 0;
    virtual std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) = 0;

  private:
};

class Test
{
  public:
    friend class UIBase;
    using Report = std::unordered_map<std::string, Action::Report>;

    Test() {};
    ~Test() {};

    virtual std::string const& getName() const = 0;
    virtual std::string const& getPlugin() const = 0;

  protected:
    virtual std::optional<std::reference_wrapper<Action>> createAction(std::string&& name,
                                                                       std::vector<std::string>&& parameters) = 0;
    virtual std::optional<std::reference_wrapper<Action>> nextAction() = 0;
    virtual void onFinish(Report const& report) = 0;
};

class Job
{
  public:
    friend class UIBase;
    using Report = std::unordered_map<std::string, Test::Report>;

    Job() {};
    ~Job() {};

  protected:
    virtual std::optional<std::reference_wrapper<Test>> createTest(std::string&& name, std::string&& plugin) = 0;
    virtual std::optional<std::reference_wrapper<Test>> nextTest() = 0;
    virtual void onFinish(Report&& report) = 0;
};

class UIBase
{
  public:
    UIBase() : work_guard(boost::asio::make_work_guard(work_context)), work_thread([this] { work_context.run(); })
    {
    }

    virtual ~UIBase()
    {
        work_guard.reset();
        work_context.stop();
        work_thread.join();
    }

    virtual int spin()
    {
        return -1;
    }

    virtual void initialize([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
    {
        std::cout << error_ << std::endl;
    }

    static std::shared_ptr<UIBase> create(std::string const& uiName)
    {
        try
        {
            creator_ = boost::dll::import_alias<Creator>(uiName, uiName,
                                                         boost::dll::load_mode::append_decorations |
                                                             boost::dll::load_mode::search_system_folders);
        }
        catch (boost::system::system_error const& e)
        {
            error_ = "Failed to load the ui: " + uiName + ": " + e.code().message();
            return std::make_shared<UIBase>();
        }

        return creator_();
    }

    bool create_job(std::string const& filename, Job& job)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            return false;
        }

        // Parse the configuration file
        boost::property_tree::ptree root;
        boost::property_tree::read_json(file, root);

        for (auto& testPair : root)
        {
            std::string testName = std::move(testPair.first);
            boost::property_tree::ptree& testNode = testPair.second;

            std::optional<std::reference_wrapper<Test>> testOptional =
                job.createTest(std::move(testName), testNode.get<std::string>("plugin"));
            if (!testOptional.has_value())
                continue;

            Test& test = testOptional.value();

            for (auto& actionPair : testNode.get_child("actions"))
            {
                std::string actionName = std::move(actionPair.first);
                boost::property_tree::ptree& paramsArray = actionPair.second;

                std::vector<std::string> parameters;
                for (auto& param : paramsArray)
                {
                    parameters.push_back(std::move(param.second.get_value<std::string>()));
                }

                test.createAction(std::move(actionName), std::move(parameters));
            }
        }

        file.close();
        return true;
    }

    void run_job(Job& job)
    {
        Job::Report jobReport;
        std::optional<std::reference_wrapper<Test>> testOptional = job.nextTest();

        while (testOptional.has_value())
        {
            Test& test = testOptional.value();
            Test::Report testReport = run_test(test);

            jobReport.emplace(test.getName(), std::move(testReport));
            testOptional = job.nextTest();
        }

        job.onFinish(std::move(jobReport));
    }

    void run_job_async(Job& job)
    {
        work_context.post([&] { run_job(job); });
    }

  private:
    using Creator = std::shared_ptr<UIBase>();
    static boost::function<Creator> creator_;
    static std::string error_;
    boost::filesystem::path binPath_ = boost::process::search_path("mil_preflight_backend");

    boost::asio::io_context work_context;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard;
    std::thread work_thread;

    Test::Report run_test(Test& test)
    {
        boost::process::ipstream childOut;
        boost::process::ipstream childErr;
        boost::process::opstream childIn;
        std::vector<std::string> args;

        boost::process::child backend = boost::process::child(
            binPath_.string(), test.getPlugin(), boost::process::std_in<childIn, boost::process::std_out> childOut,
            boost::process::std_err > childErr);

        Test::Report testReport;
        std::optional<std::reference_wrapper<Action>> actionOptional = test.nextAction();

        while (actionOptional.has_value())
        {
            Action& action = actionOptional.value();
            Action::Report actionReport = run_action(action, childOut, childErr, childIn);

            testReport.emplace(action.getName(), std::move(actionReport));
            actionOptional = test.nextAction();
        }

        if (!childIn.fail())
            childIn << EOT << std::endl;

        backend.join();
        test.onFinish(testReport);

        return testReport;
    }

    Action::Report run_action(Action& action, boost::process::ipstream& out, boost::process::ipstream& err,
                              boost::process::opstream& in)
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
                    in << action.getName() << std::endl;
                    for (std::string const& parameter : action.getParameters())
                    {
                        in << parameter << std::endl;
                    }
                    in << GS << std::endl;
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
                if (!std::getline(out, line))
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
                if (!std::getline(out, question, GS))
                {
                    actionReport.summery = "Broken pipe";
                    break;
                }

                state = State::OPTIONS;
            }
            else if (state == State::OPTIONS)
            {
                if (!std::getline(out, line, GS))
                {
                    actionReport.summery = "Broken pipe";
                    break;
                }

                if (line[0] == EOT)
                {
                    std::shared_future<int> feedback = action.onQuestion(std::move(question), std::move(options));
                    try
                    {
                        in << feedback.get() << std::endl;
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
                if (!std::getline(out, actionReport.summery))
                {
                    break;
                }

                while (std::getline(err, line))
                {
                    if (line[0] == EOT)
                        break;
                    actionReport.stderrs.push_back(std::move(line));
                }

                break;
            }
        }

        action.onFinish(actionReport);
        return actionReport;
    }
};

inline std::string UIBase::error_ = "success";
inline boost::function<UIBase::Creator> UIBase::creator_;

}  // namespace mil_preflight
