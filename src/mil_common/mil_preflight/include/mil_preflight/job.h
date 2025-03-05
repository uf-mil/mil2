#pragma once

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/process.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>

#include <filesystem>

namespace mil_preflight
{
    class JobRunner;

    class Action
    {
        public:
        friend class JobRunner;
        Action(){};
        ~Action(){};

        protected:
        std::string name_;
        std::string parameters_;
        std::vector<std::string> stdouts_;
        std::vector<std::string> stderrs_;

        virtual void onStart() = 0;
        virtual void onSuccess(std::string const& info) = 0;
        virtual void onFail(std::string const& info) = 0;
    };

    class Test
    {
        public:
        friend class JobRunner;
        Test(){};
        ~Test(){};

        std::string name_;
        std::string plugin_;
        
        protected:

        virtual std::shared_ptr<Action> nextAction() = 0;
        virtual void onFinish() = 0;
    };

    
    class Job
    {
        public:
        friend class JobRunner;
        Job(){};
        ~Job(){};

        protected:
        virtual std::shared_ptr<Test> nextTest() = 0;
        virtual void onFinish() = 0;
    };

    class JobRunner
    {
        public:
        JobRunner()
        {
            binPath_ = std::filesystem::canonical("/proc/self/exe").parent_path();
        }
        ~JobRunner()
        {

        }

        void run(std::shared_ptr<Job> job)
        {
            if(jobThread_.joinable())
                jobThread_.join();
            jobThread_ = boost::thread(boost::bind(&JobRunner::jobThreadFunc, this, job));
        }

        private:
        boost::thread jobThread_;
        std::filesystem::path binPath_;

        void jobThreadFunc(std::shared_ptr<Job> job)
        {
            // boost::interprocess::message_queue messageQueue(boost::interprocess::create_only, "mil_preflight_message_queue", 10, sizeof(int));
            while(true)
            {
                std::shared_ptr<Test> test = job->nextTest();
                if(test == nullptr)
                    break;

                boost::process::ipstream childOut;
                boost::process::ipstream childErr;                 
                boost::process::opstream childIn;
                std::vector<std::string> args;
                // args.push_back(test->plugin_);

                boost::process::child backend((binPath_ / "mil_preflight_backend").string(), test->plugin_, 
                                                boost::process::std_in < childIn, 
                                                boost::process::std_out > childOut,
                                                boost::process::std_err > childErr);

                while(true)
                {
                    std::shared_ptr<Action> action = test->nextAction();
                    if(action == nullptr)
                        break;

                    action->onStart();
                    childIn << action->parameters_ << std::endl;
                    
                    // boost::this_thread::sleep_for(boost::chrono::milliseconds(200));

                    std::vector<std::string> stdouts;
                    std::vector<std::string> stderrs;
                    std::string line;

                    bool success = true;
                    while(std::getline(childOut, line))
                    {
                        if(line[0] == char(0x06))
                        {
                            success = true;
                            break;
                        }
                        else if(line[0] == char(0x15))
                        {
                            success = false;
                            break;
                        }
                        
                        stdouts.push_back(std::move(line));
                    }

                    while(std::getline(childErr, line))
                    {
                        if(line[0] == char(0x04))
                            break;
                        stderrs.push_back(std::move(line));
                    }

                    if(success)
                        action->onSuccess("Success");
                    else
                        action->onFail("Failed");
                }

                childIn << char(0x04) << std::endl;
                backend.join();
                test->onFinish();
            }

            job->onFinish();
        }
    };
}