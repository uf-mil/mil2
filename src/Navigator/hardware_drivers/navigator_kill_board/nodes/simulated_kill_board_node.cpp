#!/ usr / bin / env c ++

#include <ros/ros.h>

#include "navigator_kill_board/simulated_kill_board.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulated_kill_board");

    ROS_INFO("Starting Navigator Simulated Kill Board");

    // Create the simulated kill board
    navigator_kill_board::SimulatedKillBoard kill_board;

    ROS_INFO("Simulated kill board initialized");
    ROS_INFO("Available services:");
    ROS_INFO("  - ~/BUTTON_FRONT_PORT");
    ROS_INFO("  - ~/BUTTON_AFT_PORT");
    ROS_INFO("  - ~/BUTTON_FRONT_STARBOARD");
    ROS_INFO("  - ~/BUTTON_AFT_STARBOARD");
    ROS_INFO("  - ~/BUTTON_REMOTE");

    ros::spin();

    return 0;
}
