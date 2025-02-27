
namespace mil_preflight
{
    class Action
    {
        public:
        Action(){};
        ~Action(){};

        // virtual void onActionFinish()
        // {
        // }
    };

    class Test
    {
        public:
        Test(){};
        ~Test(){};
        
        virtual Action* nextAction() = 0;
    };

    template<typename Derived>
    class Job
    {
        public:
        Job(){};
        ~Job(){};

        virtual Test* nextTest() = 0;
    };
}