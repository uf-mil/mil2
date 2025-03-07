#pragma once

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/process.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>

#include <filesystem>
#include <fstream>
#include <condition_variable>

#include "mil_preflight/common.h"

namespace mil_preflight
{
    // class JobRunner;
    class Question;

    class Action
    {
        public:
        friend class JobRunner;
        Action(){};
        ~Action(){};

        virtual void onStart() = 0;
        virtual void onFinish(bool success, std::string&& summery) = 0;
        virtual std::string const& getName() const = 0;
        virtual std::vector<std::string> const& getParameters() const = 0;
        virtual void onQuestion(std::shared_ptr<Question> question) = 0;

        protected:
        std::vector<std::string> stdouts_;
        std::vector<std::string> stderrs_;
    };

    class Test
    {
        public:
        friend class JobRunner;
        Test(){};
        ~Test(){};

        virtual std::shared_ptr<Action> createAction(std::string&& name, std::vector<std::string>&& parameters) = 0;
        virtual std::shared_ptr<Action> nextAction() = 0;
        virtual void onFinish() = 0;
        virtual std::string const& getName() const = 0;
        virtual std::string const& getPlugin() const = 0;
        
    };
    
    class Job
    {
        public:
        Job(){};
        ~Job(){};

        virtual std::shared_ptr<Test> createTest(std::string&& name, std::string&& plugin) = 0;
        virtual std::shared_ptr<Test> nextTest() = 0;
        virtual void onFinish() = 0;
    };

    class Question: public std::enable_shared_from_this<Question>
    {
        public:
        Question(std::string& question, std::vector<std::string>& options):
            question_(question),
            options_(options)
        {

        }

        Question(std::string&& question, std::vector<std::string>&& options):
            question_(std::move(question)),
            options_(std::move(options))
        {

        }
        

        ~Question(){}

        inline void answer(int index)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            index_ = index;
            answered_ = true;
            cond_.notify_all();
        }

        inline int ask(std::shared_ptr<Action> action)
        {
            action->onQuestion(shared_from_this());
            std::unique_lock<std::mutex> lock(mutex_);
            cond_.wait(lock, [this]{return answered_;});
            return index_;
        }

        inline std::string const& getQuestion() const { return question_; }

        inline size_t getOptionCount() const { return options_.size(); }

        inline std::string const& getOpiton(size_t index) const { return options_[index]; }

        private:
        std::string question_;
        std::vector<std::string> options_;
        int index_ = -1;
        bool answered_ = false;
        std::condition_variable cond_;
        std::mutex mutex_;
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
                    std::vector<std::string> parameters;
                    for(boost::json::value const& parameter: actionPair.value().as_array())
                    {
                        parameters.push_back(parameter.as_string().c_str());
                    }
                    test->createAction(actionPair.key(), std::move(parameters));
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
        enum class State
        {
            START,
            STDOUT,
            SUMMERY,
            QUESTION,
            OPTIONS,
            FINISH,
        };

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

                std::shared_ptr<Action> action;

                bool success = false;
                State state = State::START;
                std::vector<std::string> stdouts;
                std::vector<std::string> stderrs;
                
                std::string line;
                std::string summery;

                std::string question;
                std::vector<std::string> options;

                while(true)
                {
                    if(state == State::START)
                    {
                        action = test->nextAction();
                        if(action == nullptr)
                            break;
                        
                        action->onStart();

                        try
                        {
                            childIn << action->getName() << std::endl;
                            for(std::string const& parameter: action->getParameters())
                            {
                                childIn << parameter << std::endl;
                            }
                            childIn << GS << std::endl;
                        }
                        catch(const std::exception& e)
                        {
                            state = State::FINISH;
                            summery = "Broken pipe: " + std::string(e.what());
                            continue;
                        }
                        state = State::STDOUT;
                    }
                    else if(state == State::STDOUT)
                    {
                        if(!std::getline(childOut, line))
                        {
                            summery = "Broken pipe";
                            state = State::FINISH;
                            continue;
                        }

                        if(line[0] == ACK)
                        {
                            state = State::SUMMERY;
                            success = true;
                        }
                        else if(line[0] == NCK)
                        {
                            state = State::SUMMERY;
                            success = false;
                        }
                        else if(line[0] == BEL)
                        {
                            state = State::QUESTION;
                        }
                        else
                        {
                            stdouts.push_back(std::move(line));
                        }
                    }
                    else if(state == State::QUESTION)
                    {
                        if(!std::getline(childOut, question))
                        {
                            summery = "Broken pipe";
                            state = State::FINISH;
                            continue;
                        }

                        state = State::OPTIONS;
                    }
                    else if(state == State::OPTIONS)
                    {
                        if(!std::getline(childOut, line))
                        {
                            summery = "Broken pipe";
                            state = State::FINISH;
                            continue;
                        }

                        if(line[0] == EOT)
                        {
                            std::shared_ptr<Question> q = std::make_shared<Question>(std::move(question), std::move(options));
                            try
                            {
                                childIn << q->ask(action) << std::endl;
                            }
                            catch(const std::exception& e)
                            {
                                state = State::FINISH;
                                summery = "Broken pipe: " + std::string(e.what());
                                continue;
                            }
                            state = State::STDOUT;
                        }
                        else
                        {
                            options.push_back(std::move(line));
                        }

                    }
                    else if(state == State::SUMMERY)
                    {
                        if(!std::getline(childOut, summery))
                        {
                            state = State::FINISH;
                            continue;
                        }

                        state = State::FINISH;
                    }
                    else if(state == State::FINISH)
                    {
                        action->onFinish(success, std::move(summery));
                        stdouts.clear();
                        stderrs.clear();
                        state = State::START;
                    }
                }

                // while(std::getline(childErr, line))
                // {
                //     if(line[0] == EOT)
                //         break;
                //     stderrs.push_back(std::move(line));
                // }

                if(!childIn.fail())
                    childIn << EOT << std::endl;
                
                backend.join();
                test->onFinish();
            }

            job->onFinish();
        }
    };
}