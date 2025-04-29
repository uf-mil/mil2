#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "subjugator_trajectory_planner/TrajectoryPlanner.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}
