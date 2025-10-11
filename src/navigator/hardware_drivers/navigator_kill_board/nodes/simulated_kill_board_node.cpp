#include <rclcpp/rclcpp.hpp>

#include "simulated_kill_board.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "Starting Navigator Simulated Kill Board");

    // Create the simulated kill board
    navigator_kill_board::SimulatedKillBoard kill_board;

    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "Simulated kill board initialized");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "Available services:");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_FRONT_PORT");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_AFT_PORT");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_FRONT_STARBOARD");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_AFT_STARBOARD");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_REMOTE");

    rclcpp::spin(kill_board.get_node());

    rclcpp::shutdown();
    return 0;
}
