/**
 * @file pcl_tracker_node.cpp
 * @brief Entry point for the pcl_tracker ROS 2 node.
 *
 * Spins a PclTracker instance that subscribes to "cluster_markers" and
 * publishes "tracked_markers" with stable per-object EKF track IDs.
 */

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcl_tracker.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcd::PclTracker>());
    rclcpp::shutdown();
    return 0;
}
