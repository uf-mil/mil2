#pragma once

#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <optional>
#include <string>
#include <unordered_map>

#include <boost/chrono.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "mil_preflight/common.h"

namespace mil_preflight
{

class Job;
class Test;

class Action
{
  public:
    friend class Test;
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
    enum class State
    {
        START,
        STDOUT,
        SUMMERY,
        QUESTION,
        OPTIONS,
    };

    Action::Report run(boost::process::ipstream& out, boost::process::ipstream& err, boost::process::opstream& in)
    {
        onStart();

        Action::Report actionReport;
        State state = State::START;

        std::string line;

        std::string question;
        std::vector<std::string> options;

        while (true)
        {
            if (state == State::START)
            {
                try
                {
                    in << getName() << std::endl;
                    for (std::string const& parameter : getParameters())
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
                    std::shared_future<int> feedback = onQuestion(std::move(question), std::move(options));
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

        onFinish(actionReport);
        return actionReport;
    }
};

class Test
{
  public:
    friend class Job;
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

  private:
    boost::filesystem::path binPath_ = boost::process::search_path("mil_preflight_backend");

    Test::Report run()
    {
        boost::process::ipstream childOut;
        boost::process::ipstream childErr;
        boost::process::opstream childIn;
        std::vector<std::string> args;

        boost::process::child backend = boost::process::child(
            binPath_.string(), getPlugin(), boost::process::std_in<childIn, boost::process::std_out> childOut,
            boost::process::std_err > childErr);

        Test::Report testReport;
        std::optional<std::reference_wrapper<Action>> actionOptional = nextAction();

        while (actionOptional.has_value())
        {
            Action& action = actionOptional.value();
            Action::Report actionReport = action.run(childOut, childErr, childIn);

            testReport.emplace(action.getName(), std::move(actionReport));
            actionOptional = nextAction();
        }

        if (!childIn.fail())
            childIn << EOT << std::endl;

        backend.join();
        onFinish(testReport);

        return testReport;
    }
};

class Job
{
  public:
    using Report = std::unordered_map<std::string, Test::Report>;

    Job() {};
    Job(std::string const& filePath)
    {
        initialize(filePath);
    }
    ~Job()
    {
        if (future_.valid())
            future_.wait();
    };

    bool initialize(std::string const& filePath)
    {
        std::ifstream file(filePath);
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
                createTest(std::move(testName), testNode.get<std::string>("plugin"));
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

    void runAsync()
    {
        future_ = std::async(std::launch::async, std::bind(&Job::run, this));
    }

    void run()
    {
        Job::Report jobReport;
        std::optional<std::reference_wrapper<Test>> testOptional = nextTest();

        while (testOptional.has_value())
        {
            Test& test = testOptional.value();
            Test::Report testReport = test.run();

            jobReport.emplace(test.getName(), std::move(testReport));
            testOptional = nextTest();
        }

        onFinish(std::move(jobReport));
    }

    void cancel()
    {
    }

    bool isRunning()
    {
        if (!future_.valid())
            return false;

        if (future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            return false;

        return true;
    }

  protected:
    virtual std::optional<std::reference_wrapper<Test>> createTest(std::string&& name, std::string&& plugin) = 0;
    virtual std::optional<std::reference_wrapper<Test>> nextTest() = 0;
    virtual void onFinish(Report&& report) = 0;

  private:
    std::future<void> future_;
};
}  // namespace mil_preflight
