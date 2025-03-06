#pragma once

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/process.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>

#include <filesystem>
#include <fstream>

#include "mil_preflight/common.h"

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
        std::vector<std::string> stdouts_;
        std::vector<std::string> stderrs_;

        virtual void onStart() = 0;
        virtual void onFinish(bool success, std::string const& summery) = 0;
        virtual std::string const& getName() const = 0;
        virtual std::string const& getParameter() const = 0;
    };

    class Test
    {
        public:
        friend class JobRunner;
        Test(){};
        ~Test(){};
        
        protected:

        virtual std::shared_ptr<Action> createAction(std::string const& name, std::string const& parameters) = 0;
        virtual std::shared_ptr<Action> nextAction() = 0;
        virtual void onFinish() = 0;
        virtual std::string const& getName() const = 0;
        virtual std::string const& getPlugin() const = 0;
    };

    
    class Job
    {
        public:
        friend class JobRunner;
        Job(){};
        ~Job(){};

        protected:
        virtual std::shared_ptr<Test> createTest(std::string const& name, std::string const& plugin) = 0;
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

        bool initialize(std::shared_ptr<Job> job, std::string const& filePath)
        {
            std::ifstream file(filePath);
            if(!file.is_open())
            {
                return false;
            }

            // Parse the configuration file
            boost::json::value data = boost::json::parse(file);
            for(boost::json::key_value_pair const& testPair : data.as_object())
            {
                std::shared_ptr<Test> test = job->createTest(testPair.key(), testPair.value().at("plugin").as_string().c_str());
                for(boost::json::key_value_pair const& actionPair : testPair.value().at("actions").as_object())
                {
                    test->createAction(actionPair.key(), actionPair.value().as_string().c_str());
                }
            }

            file.close();
            return true;
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
            while(true)
            {
                std::shared_ptr<Test> test = job->nextTest();
                if(test == nullptr)
                    break;

                boost::process::ipstream childOut;
                boost::process::ipstream childErr;                 
                boost::process::opstream childIn;
                std::vector<std::string> args;

                boost::process::child backend((binPath_ / "mil_preflight_backend").string(), test->getPlugin(), 
                                                boost::process::std_in < childIn, 
                                                boost::process::std_out > childOut,
                                                boost::process::std_err > childErr);


                while(true)
                {
                    bool success = false;
                    std::shared_ptr<Action> action = test->nextAction();
                    if(action == nullptr)
                        break;

                    action->onStart();
                    childIn << action->getParameter() << std::endl;

                    std::vector<std::string> stdouts;
                    std::vector<std::string> stderrs;
                    
                    std::string line;
                    while(std::getline(childOut, line))
                    {
                        if(line[0] == ACK)
                        {
                            success = true;
                            break;
                        }
                        else if(line[0] == NCK)
                        {
                            success = false;
                            break;
                        }
                        else if(line[0] == BEL)
                        {

                        }
                        
                        stdouts.push_back(std::move(line));
                    }

                    while(std::getline(childErr, line))
                    {
                        if(line[0] == EOT)
                            break;
                        stderrs.push_back(std::move(line));
                    }

                    action->onFinish(success, line.substr(1, line.size() - 1));

                }

                childIn << EOT << std::endl;
                backend.join();
                test->onFinish();
            }

            job->onFinish();
        }
    };
}