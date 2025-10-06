#include "subjugator_controller/controller.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}
