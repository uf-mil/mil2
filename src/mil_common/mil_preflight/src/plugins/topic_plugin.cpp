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

        bool runAction(std::string const& parameter) final
        {
            return topics_.find(parameter) != topics_.end();
        }

        static std::shared_ptr<TopicPlugin> create() 
        {
            return std::shared_ptr<TopicPlugin>(new TopicPlugin());
        }

        private:
        std::map<std::string, std::vector<std::string>> topics_;
    };
}


BOOST_DLL_ALIAS(mil_preflight::TopicPlugin::create, topic_plugin);

