#include <rclcpp/rclcpp.hpp>

#include <filesystem>

#include "mil_preflight/plugin.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc - 1, argv);

    std::filesystem::path binPath = std::filesystem::canonical("/proc/self/exe").parent_path();
    std::filesystem::path pluginPath = binPath/".."/"lib"/ argv[argc-1];

    std::shared_ptr<rclcpp::Node> node = std::make_shared<mil_preflight::PluginBase>(argv[argc - 1]);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}