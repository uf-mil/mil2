#include <map>

#include <boost/dll/alias.hpp>

#include "mil_preflight/plugin.h"

namespace mil_preflight
{
class TopicPlugin : public SimplePlugin
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

    std::pair<bool, std::string> runAction(std::string const& name, std::vector<std::string> const& parameters) final
    {
        auto it = topics_.find(parameters[0]);
        if (it == topics_.end())
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            topics_ = get_topic_names_and_types();

            it = topics_.find(parameters[0]);
            if (it == topics_.end())
            {
                return { false, "Topic " + name + " does not exist in 100 ms" };
            }
        }

        return { true, "Found topic " + name + " with type " + it->second[0] };
    }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::TopicPlugin::create, topic_plugin);
