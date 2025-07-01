#include "mil_preflight/ui.h"

using Creator = std::shared_ptr<mil_preflight::UIBase>();
std::function<Creator> load(std::string const& uiName)
{
    try
    {
        return boost::dll::import_alias<Creator>(
            uiName, uiName, boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders);
    }
    catch (boost::system::system_error const& e)
    {
        return []() -> std::shared_ptr<mil_preflight::UIBase> { return std::make_shared<mil_preflight::UIBase>(); };
    }
}

int main(int argc, char* argv[])
{
    if (argc <= 1)
    {
        std::cout << "Usage: mil_preflight <ui> [ui_parameters...]" << std::endl;
        return -1;
    }

    std::function<Creator> creator = load(argv[1]);
    std::shared_ptr<mil_preflight::UIBase> ui = creator();
    std::shared_ptr<mil_preflight::Frontend> frontend = std::make_shared<mil_preflight::Frontend>(argc - 1, &argv[1]);

    return ui->run(frontend);
}
