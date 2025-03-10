#pragma once

#include <boost/chrono.hpp>
#include <boost/process.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/json.hpp>

#include <filesystem>
#include <fstream>
#include <condition_variable>
#include <string>
#include <future>
#include <unordered_map>
#include <optional>

#include "mil_preflight/common.h"

namespace mil_preflight
{

    class Action
    {
        public:

        struct Report
        {
            bool success;
            std::string summery;
            std::vector<std::string> stdouts;
            std::vector<std::string> stderrs;

            Report():success(false){}
            ~Report() = default;
            Report(Report const& report) = default;
            Report(Report && report)
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

        class Feedback
        {
            public:
            Feedback():impl_(std::make_shared<Impl>())
            {

            }

            Feedback(Feedback&& feedback){impl_ = std::move(feedback.impl_);}
            Feedback(Feedback const& feedback){impl_ = feedback.impl_;}
            Feedback& operator=(Feedback&& feedback) 
            {
                impl_ = std::move(feedback.impl_);
                return *this;
            }
            Feedback& operator=(Feedback const& feedback)
            {
                impl_ = feedback.impl_;
                return *this;
            }

            ~Feedback(){}

            void set(int index)
            {
                std::unique_lock<std::mutex> lock(impl_->mutex_);
                if(!impl_->answered_)
                {
                    impl_->index_ = index;
                    impl_->answered_ = true;
                    impl_->cond_.notify_all();
                }
            }

            int get() const
            {
                std::unique_lock<std::mutex> lock(impl_->mutex_);
                impl_->cond_.wait(lock, [this]{return impl_->answered_;});
                return impl_->index_;
            }

            private:
            struct Impl
            {
                int index_ = -1;
                bool answered_ = false;
                std::condition_variable cond_;
                std::mutex mutex_;
            };

            std::shared_ptr<Impl> impl_;
        };

        Action(){};
        ~Action(){};

        virtual void onStart() = 0;
        virtual void onFinish(Report const& report) = 0;
        virtual std::string const& getName() const = 0;
        virtual std::vector<std::string> const& getParameters() const = 0;
        virtual std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) = 0;

        protected:
        std::vector<std::string> stdouts_;
        std::vector<std::string> stderrs_;
    };

    class Test
    {
        public:
        using Report = std::unordered_map<std::string, Action::Report>;

        Test(){};
        ~Test(){};

        virtual std::optional<std::reference_wrapper<Action>> createAction(std::string&& name, std::vector<std::string>&& parameters) = 0;
        virtual std::optional<std::reference_wrapper<Action>> nextAction() = 0;
        virtual void onFinish(Report const& report) = 0;
        virtual std::string const& getName() const = 0;
        virtual std::string const& getPlugin() const = 0;
        
    };

    class Question
    {
        public:
        Question(){}
        ~Question()
        {
            answer(-1);
        }

        inline void answer(int index)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if(!answered_)
            {
                index_ = index;
                answered_ = true;
                cond_.notify_all();
            }
        }

        inline int ask()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_.wait(lock, [this]{return answered_;});
            return index_;
        }

        private:
        int index_ = -1;
        bool answered_ = false;
        std::condition_variable cond_;
        std::mutex mutex_;
    };
    
    class Job
    {
        public:
        using Report = std::unordered_map<std::string, Test::Report>;

        Job(){};
        Job(std::string const& filePath){ initialize(filePath); }
        ~Job(){};

        bool initialize(std::string const& filePath)
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
                std::optional<std::reference_wrapper<Test>> testOptional = createTest(testPair.key(), testPair.value().at("plugin").as_string().c_str());
                if(!testOptional.has_value())
                    continue;

                Test& test = testOptional.value();
                
                for(boost::json::key_value_pair const& actionPair : testPair.value().at("actions").as_object())
                {
                    std::vector<std::string> parameters;
                    for(boost::json::value const& parameter: actionPair.value().as_array())
                    {
                        parameters.push_back(parameter.as_string().c_str());
                    }
                    test.createAction(actionPair.key(), std::move(parameters));
                }
            }

            file.close();
            return true;
        }

        void run()
        {
            if(isRunning())
                return;
            
            future_ = std::async(std::launch::async, std::bind(&Job::runJob, this));
        }

        void cancel()
        {
            if(backend_.valid() && backend_.joinable())
                backend_.terminate();
        }

        bool isRunning()
        {
            if(!future_.valid())
                return false;
            
            if(future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
                return false;
            
            return true;
        }

        protected:

        virtual std::optional<std::reference_wrapper<Test>> createTest(std::string&& name, std::string&& plugin) = 0;
        virtual std::optional<std::reference_wrapper<Test>> nextTest() = 0;
        virtual void onFinish(Report&& report) = 0;

        private:
        enum class State
        {
            START,
            STDOUT,
            SUMMERY,
            QUESTION,
            OPTIONS,
        };

        std::future<void> future_;
        boost::process::child backend_;
        std::filesystem::path binPath_ = std::filesystem::canonical("/proc/self/exe").parent_path();

        void runJob()
        {
            Job::Report jobReport;
            while(true)
            {
                std::optional<std::reference_wrapper<Test>> testOptional = nextTest();
                if(!testOptional.has_value())
                    break;

                Test& test = testOptional.value();

                boost::process::ipstream childOut;
                boost::process::ipstream childErr;                 
                boost::process::opstream childIn;
                std::vector<std::string> args;

                backend_ =  boost::process::child((binPath_ / "mil_preflight_backend").string(), test.getPlugin(), 
                            boost::process::std_in < childIn, 
                            boost::process::std_out > childOut,
                            boost::process::std_err > childErr);

                
                std::string line;

                std::string question;
                std::vector<std::string> options;

                
                Test::Report testReport;
                while(true)
                {
                    std::optional<std::reference_wrapper<Action>> actionOptional = test.nextAction();
                    if(!actionOptional.has_value())
                        break;
                    
                    Action& action = actionOptional.value();
                    action.onStart();

                    Action::Report actionReport;
                    State state = State::START;

                    while(true)
                    {
                        if(state == State::START)
                        {
                            try
                            {
                                childIn << action.getName() << std::endl;
                                for(std::string const& parameter: action.getParameters())
                                {
                                    childIn << parameter << std::endl;
                                }
                                childIn << GS << std::endl;
                            }
                            catch(const std::exception& e)
                            {
                                actionReport.summery = "Broken pipe: " + std::string(e.what());
                                break;
                            }
                            state = State::STDOUT;
                        }
                        else if(state == State::STDOUT)
                        {
                            if(!std::getline(childOut, line))
                            {
                                actionReport.summery = "Broken pipe";
                                break;
                            }

                            if(line[0] == ACK)
                            {
                                state = State::SUMMERY;
                                actionReport.success = true;
                            }
                            else if(line[0] == NCK)
                            {
                                state = State::SUMMERY;
                            }
                            else if(line[0] == BEL)
                            {
                                state = State::QUESTION;
                            }
                            else
                            {
                                actionReport.stdouts.push_back(std::move(line));
                            }
                        }
                        else if(state == State::QUESTION)
                        {
                            if(!std::getline(childOut, question, GS))
                            {
                                actionReport.summery = "Broken pipe";
                                break;
                            }

                            state = State::OPTIONS;
                        }
                        else if(state == State::OPTIONS)
                        {
                            if(!std::getline(childOut, line, GS))
                            {
                                actionReport.summery = "Broken pipe";
                                break;
                            }

                            if(line[0] == EOT)
                            {
                                std::shared_future<int> feedback = action.onQuestion(std::move(question), std::move(options));
                                try
                                {
                                    childIn << feedback.get() << std::endl;
                                    options.clear();
                                }
                                catch(const std::exception& e)
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
                        else if(state == State::SUMMERY)
                        {
                            if(!std::getline(childOut, actionReport.summery))
                            {
                                break;
                            }

                            break;
                        }
                    }

                    action.onFinish(actionReport);
                    testReport.emplace(action.getName(), std::move(actionReport));
                }

                // while(std::getline(childErr, line))
                // {
                //     if(line[0] == EOT)
                //         break;
                //     stderrs.push_back(std::move(line));
                // }

                if(!childIn.fail())
                    childIn << EOT << std::endl;
                
                backend_.join();
                test.onFinish(testReport);
                jobReport.emplace(test.getName(), std::move(testReport));
            }

            onFinish(std::move(jobReport));
        }
    };
}