#include "mil_preflight/plugin.h"

namespace mil_preflight
{
    class SetupPlugin: public PluginBase
    {
        public:

        SetupPlugin()
        {
            
        }

        ~SetupPlugin()
        {

        }

        static std::shared_ptr<SetupPlugin> create() 
        {
            return std::shared_ptr<SetupPlugin>(new SetupPlugin());
        }

        private:

        std::map<std::string, std::vector<std::string>> topics_;
        std::string summery_;

        bool runAction(std::string const& parameter) final
        {
            if(askUser(parameter, {"Yes", "No"}) != 0)
            {
                summery_ = "User said No";
                return false;
            }

            summery_ = "success";
            return true;
        }

        std::string const& getSummery() final
        {
            return summery_;
        }
    };
}


BOOST_DLL_ALIAS(mil_preflight::SetupPlugin::create, setup_plugin);