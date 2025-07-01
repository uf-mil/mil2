#include <filesystem>

#include <boost/dll.hpp>
#include <boost/function.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_preflight/common.h"
#include "mil_preflight/plugin.h"

using Creator = std::shared_ptr<mil_preflight::PluginBase>();
boost::function<Creator> load(std::string const& plugin_name)
{
    try
    {
        return boost::dll::import_alias<Creator>(plugin_name, plugin_name,
                                                 boost::dll::load_mode::append_decorations |
                                                     boost::dll::load_mode::search_system_folders);
    }
    catch (boost::system::system_error const& e)
    {
        return []() -> std::shared_ptr<mil_preflight::PluginBase>
        { return std::make_shared<mil_preflight::PluginBase>(); };
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    std::unordered_map<std::string, boost::function<Creator>> libraries;

    std::thread io_thread(
        [&]
        {
            std::string line;
            std::shared_ptr<mil_preflight::PluginBase> node;
            while (std::getline(std::cin, line))
            {
                if (line[0] == mil_preflight::EOT)
                {
                    rclcpp::shutdown();
                    return;
                }

                auto it = libraries.find(line);
                if (it == libraries.end())
                {
                    std::function<Creator> creator = load(line);
                    it = libraries.emplace(std::move(line), std::move(creator)).first;
                }

                node = it->second();
                exec.add_node(node);
                node->runTest();
                exec.remove_node(node);
                node.reset();
            }
        });

    exec.spin();
    rclcpp::shutdown();
    io_thread.join();
    return 0;
}
