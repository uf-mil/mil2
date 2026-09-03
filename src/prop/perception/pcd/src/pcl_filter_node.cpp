/**
 * @file pcl_filter_node.cpp
 * @brief Entry point for the pcl_filter ROS 2 node.
 */

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcl_filter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcd::PclFilter>());
    rclcpp::shutdown();
    return 0;
}
