/**
 * @file pcl_clustering_node.cpp
 * @brief Entry point for the pcl_clustering ROS 2 node.
 */

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcl_clustering.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcd::PclClustering>());
    rclcpp::shutdown();
    return 0;
}
