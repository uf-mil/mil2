
#include "mil_preflight/frontend.h"

#include <boost/function.hpp>

namespace mil_preflight
{

Frontend::Frontend(int argc, char* argv[])
  : work_guard_(boost::asio::make_work_guard(work_context_))
  , work_thread_([this] { work_context_.run(); })
  , backend_(boost::process::child(bin_path_.string(),
                                   boost::process::std_in<child_in_, boost::process::std_out> child_out_,
                                   boost::process::std_err > child_err_))
{
    for (int i = 0; i < argc; i++)
    {
        args_.emplace_back(argv[i]);
    }
}

Frontend::~Frontend()
{
    work_guard_.reset();
    work_context_.stop();
    work_thread_.join();

    if (backend_.running())
        child_in_ << EOT << std::endl;

    backend_.join();
}

void Frontend::runJob(std::shared_ptr<Job> job)
{
    std::shared_ptr<Test> test = job->nextTest();

    while (test != nullptr)
    {
        runTest(test);
        test = job->nextTest();
    }

    job->onFinish();
}

void Frontend::runJobAsync(std::shared_ptr<Job> job)
{
    work_context_.post([=] { runJob(job); });
}

void Frontend::runTest(std::shared_ptr<Test> test)
{
    std::shared_ptr<Action> action = test->nextAction();

    child_in_ << test->getName() << std::endl << test->getPlugin() << std::endl << GS << std::endl;

    while (action != nullptr)
    {
        runAction(action);
        action = test->nextAction();
    }

    if (!child_in_.fail())
        child_in_ << EOT << std::endl;

    test->onFinish();
}

void Frontend::runAction(std::shared_ptr<Action> action)
{
    action->onStart();

    enum class State
    {
        START,
        STDOUT,
        SUMMERY,
        QUESTION,
        OPTIONS,
    } state = State::START;

    std::string line;
    std::string summery;
    bool success = false;
    action->stdouts.clear();
    action->stderrs.clear();

    std::string question;
    std::vector<std::string> options;

    while (true)
    {
        if (state == State::START)
        {
            try
            {
                child_in_ << action->getName() << std::endl;
                for (std::string const& parameter : action->getParameters())
                {
                    child_in_ << parameter << std::endl;
                }
                child_in_ << GS << std::endl;
            }
            catch (std::exception const& e)
            {
                summery = "Broken pipe: " + std::string(e.what());
                break;
            }
            state = State::STDOUT;
        }
        else if (state == State::STDOUT)
        {
            if (!std::getline(child_out_, line))
            {
                summery = "Broken pipe";
                break;
            }

            if (line[0] == ACK)
            {
                state = State::SUMMERY;
                success = true;
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
                action->stdouts.push_back(std::move(line));
            }
        }
        else if (state == State::QUESTION)
        {
            if (!std::getline(child_out_, question, GS))
            {
                summery = "Broken pipe";
                break;
            }

            state = State::OPTIONS;
        }
        else if (state == State::OPTIONS)
        {
            if (!std::getline(child_out_, line, GS))
            {
                summery = "Broken pipe";
                break;
            }

            if (line[0] == EOT)
            {
                std::shared_future<int> feedback = action->onQuestion(std::move(question), std::move(options));
                try
                {
                    child_in_ << feedback.get() << std::endl;
                    options.clear();
                }
                catch (std::exception const& e)
                {
                    summery = "Broken pipe: " + std::string(e.what());
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
            if (!std::getline(child_out_, summery))
            {
                break;
            }

            while (std::getline(child_err_, line))
            {
                if (line[0] == EOT)
                    break;
                action->stderrs.push_back(std::move(line));
            }

            break;
        }
    }

    action->onFinish(success, std::move(summery));
}

}  // namespace mil_preflight
