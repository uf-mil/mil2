#include "subjugator_controller/controller.h"

PIDController::PIDController() : Node("pid_controller")
{
    auto odom_callback = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void { std::cout << "hello" << std::endl; };
    subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);
}
