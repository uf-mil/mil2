#include "subjugator_keyboard_control/SubjugatorKeyboardControl.h"

int main(int const argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubjugatorKeyboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
