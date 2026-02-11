#include <map>
#include <vector>

#include "mil_preflight/plugin.h"

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

    std::pair<bool, std::string> runAction(std::string const& name, std::vector<std::string> const& parameters) final
    {
        if (std::find(nodes_.begin(), nodes_.end(), parameters[0]) == nodes_.end())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            nodes_ = get_node_names();
            if (std::find(nodes_.begin(), nodes_.end(), parameters[0]) == nodes_.end())
            {
                return { false, "Node " + name + " does not exist in 100 ms" };
            }
        }

        return { true, "Found node " + parameters[0] };
    }
};
}  // namespace mil_preflight

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mil_preflight::NodePlugin, mil_preflight::PluginBase)
