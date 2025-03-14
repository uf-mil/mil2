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
    // boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
    if (topics_.find(parameters[1]) == topics_.end())
    {
      summery_ = "Topic " + parameters[0] + " does not exist";
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
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::TopicPlugin::create, topic_plugin);
