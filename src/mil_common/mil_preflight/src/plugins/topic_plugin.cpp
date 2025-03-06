#include "mil_preflight/plugin.h"

#include <map>

#include <boost/dll/alias.hpp>

namespace mil_preflight
{
    class TopicPlugin: public PluginBase
    {
        public:

        TopicPlugin()
        {
            topics_ = get_topic_names_and_types();
        }

        static std::shared_ptr<TopicPlugin> create() 
        {
            return std::shared_ptr<TopicPlugin>(new TopicPlugin());
        }

        private:

        std::map<std::string, std::vector<std::string>> topics_;
        std::string summery_;

        bool runAction(std::string const& parameter) final
        {
            if(topics_.find(parameter) == topics_.end())
            {
                summery_ = "Topic " + parameter + " does not exist";
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


BOOST_DLL_ALIAS(mil_preflight::TopicPlugin::create, topic_plugin);

