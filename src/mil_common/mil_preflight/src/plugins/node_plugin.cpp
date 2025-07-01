#include <map>
#include <vector>

#include <boost/dll/alias.hpp>

#include "mil_preflight/backend.h"

namespace mil_preflight
{
class NodePlugin : public SimplePlugin
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

    std::pair<bool, std::string> runAction(std::shared_ptr<Action> action) final
    {
        std::string const& node_name = action->getParameters()[0];
        if (std::find(nodes_.begin(), nodes_.end(), node_name) == nodes_.end())
        {
            nodes_ = get_node_names();
            if (std::find(nodes_.begin(), nodes_.end(), node_name) == nodes_.end())
            {
                return { false, "Node " + action->getName() + " does not exist in 100 ms" };
            }
        }

        return { true, "Found node " + action->getName() };
    }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::NodePlugin::create, node_plugin);
