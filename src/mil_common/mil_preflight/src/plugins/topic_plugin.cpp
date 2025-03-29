#include <boost/dll/alias.hpp>
#include <map>

#include "mil_preflight/plugin.h"

namespace mil_preflight
{
class TopicPlugin : public PluginBase
{
  public:
    TopicPlugin()
    {
        topics_ = get_topic_names_and_types();
    }

    ~TopicPlugin()
    {
    }

    static std::shared_ptr<TopicPlugin> create()
    {
        return std::shared_ptr<TopicPlugin>(new TopicPlugin());
    }

  private:
    std::map<std::string, std::vector<std::string>> topics_;
    std::string summery_;
    bool runAction(std::vector<std::string>&& parameters) final
    {
        auto it = topics_.find(parameters[1]);
        if (it == topics_.end())
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            topics_ = get_topic_names_and_types();

            it = topics_.find(parameters[1]);
            if (it == topics_.end())
            {
                summery_ = "Topic " + parameters[0] + " does not exist in 100 ms";
                return false;
            }
        }

        summery_ = "Found topic " + parameters[0] + " with type " + it->second[0];
        return true;
    }

    std::string const& getSummery() final
    {
        return summery_;
    }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::TopicPlugin::create, topic_plugin);
