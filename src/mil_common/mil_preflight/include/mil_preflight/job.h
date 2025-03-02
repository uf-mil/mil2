#include <boost/thread.hpp>
#include <boost/chrono.hpp>

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

        void jobThreadFunc(std::shared_ptr<Job> job)
        {
            while(true)
            {
                std::shared_ptr<Test> test = job->nextTest();
                if(test == nullptr)
                    break;
                
                // Create process here
                while(true)
                {
                    std::shared_ptr<Action> action = test->nextAction();
                    if(action == nullptr)
                        break;

                    action->onStart();
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                    // Send the action to the process
                    // Wait for the process run the action
                    action->onFail("Failed");
                }

                test->onFinish();
            }

            job->onFinish();
        }
    };
}