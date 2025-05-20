#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "subjugator_path_planner/PathPlanner.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
