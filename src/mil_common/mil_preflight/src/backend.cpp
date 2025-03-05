#include <rclcpp/rclcpp.hpp>
#include <boost/dll.hpp>
#include <boost/function.hpp>

#include <filesystem>

#include "mil_preflight/plugin.h"




int main(int argc, char* argv[])
{
    if(argc <= 1)
        return 1;

    std::filesystem::path binPath = std::filesystem::canonical("/proc/self/exe").parent_path();
    std::string pluginName = argv[argc-1];
    std::filesystem::path pluginPath = binPath/".."/"lib"/pluginName;


    using Creator = std::shared_ptr<mil_preflight::PluginBase>();
    boost::function<Creator> creator;
    bool error = false;
    try
    {
        creator = boost::dll::import_alias<Creator>(
            pluginPath.string(),
            pluginName,
            boost::dll::load_mode::append_decorations
        );
    }
    catch(boost::system::system_error const& e)
    {
        error = true;
    }

    rclcpp::init(argc - 1, argv);
    std::shared_ptr<mil_preflight::PluginBase> node;
    if(!error)
        node = creator();
    else
        node = std::make_shared<mil_preflight::PluginBase>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}