#!/ usr / bin / env c ++

#include <ros/ros.h>

#include "navigator_kill_board/heartbeat_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "heartbeat_server");

    // Create heartbeat server publishing to /network topic with 1.0 second period
    navigator_kill_board::HeartbeatServer server("/network", 1.0);

    ROS_INFO("Heartbeat server started, publishing to /network");

    ros::spin();

    return 0;
}
