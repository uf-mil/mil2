#include "mil_preflight/plugin.h"

#include <map>
#include <vector>

#include <boost/dll/alias.hpp>

namespace mil_preflight
{
    class ActuatorPlugin: public PluginBase
    {
        public:

        ActuatorPlugin()
        {
            // nodes_ = get_node_names();
        }

        ~ActuatorPlugin()
        {
            
        }

        static std::shared_ptr<ActuatorPlugin> create() 
        {
            return std::shared_ptr<ActuatorPlugin>(new ActuatorPlugin());
        }

        private:

        std::vector<std::string> nodes_;
        std::string summery_;

        bool runAction(std::vector<std::string>&& parameters) final
        {
            std::string question = "Ensure that all fingers are clear of the area! Is it safe to operate the actuator: " + 
                parameters[0] + " ?";
            if(askQuestion(question, {"Yes", "No"}) != 0)
            {
                summery_ = "User did not clear the area";
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


BOOST_DLL_ALIAS(mil_preflight::ActuatorPlugin::create, actuator_plugin);

