#include <vector>

#include <boost/dll.hpp>

#include "mil_preflight/backend.h"

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

    std::pair<bool, std::string> runAction(std::shared_ptr<Action> action) final
    {
        std::string question = action->getParameters()[0];
        if (action->onQuestion(std::move(question), { "Yes", "No" }).get() != 0)
        {
            return { false, "User said No" };
        }

        return { true, "Success" };
    }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::SetupPlugin::create, setup_plugin);
