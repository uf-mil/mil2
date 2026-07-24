#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "subjugator_movement_manager/MovementManager.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementManager>());
    rclcpp::shutdown();
    return 0;
}
