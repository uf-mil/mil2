#include <boost/dll/alias.hpp>
#include <map>
#include <vector>

#include "mil_preflight/plugin.h"

namespace mil_preflight
{
class NodePlugin : public PluginBase
{
public:
  NodePlugin()
  {
    nodes_ = get_node_names();
  }

  ~NodePlugin()
  {
  }

  static std::shared_ptr<NodePlugin> create()
  {
    return std::shared_ptr<NodePlugin>(new NodePlugin());
  }

private:
  std::vector<std::string> nodes_;
  std::string summery_;

  bool runAction(std::vector<std::string>&& parameters) final
  {
    if (std::find(nodes_.begin(), nodes_.end(), parameters[1]) == nodes_.end())
    {
      nodes_ = get_node_names();
      if (std::find(nodes_.begin(), nodes_.end(), parameters[1]) == nodes_.end())
      {
        summery_ = "Node " + parameters[0] + " does not exist in 100 ms";
        return false;
      }
    }

    summery_ = "Found node " + parameters[1];

    return true;
  }

  std::string const& getSummery() final
  {
    return summery_;
  }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::NodePlugin::create, node_plugin);
