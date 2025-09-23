#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_planner");
    RCLCPP_INFO(node->get_logger(), "Mission Planner package wired up (Step 1).");
    rclcpp::shutdown();
    return 0;
}
