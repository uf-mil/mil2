#include <vector>

#include "mil_preflight/plugin.h"

namespace mil_preflight
{
class SetupPlugin : public SimplePlugin
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

    std::pair<bool, std::string> runAction([[maybe_unused]] std::string const& name,
                                           std::vector<std::string> const& parameters) final
    {
        if (askQuestion(std::string(parameters[0]), { "Yes", "No" }) != 0)
        {
            return { false, "User said No" };
        }

        return { true, "Success" };
    }
};
}  // namespace mil_preflight

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mil_preflight::SetupPlugin, mil_preflight::PluginBase)
