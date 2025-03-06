#include <rclcpp/rclcpp.hpp>
#include <boost/dll.hpp>
#include <boost/function.hpp>

#include <filesystem>

#include "mil_preflight/plugin.h"

int main(int argc, char* argv[])
{
    if(argc <= 1)
        return 1;

    rclcpp::init(argc - 1, argv);
    std::shared_ptr<mil_preflight::PluginBase> node = mil_preflight::PluginBase::create(argv[argc - 1]);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}