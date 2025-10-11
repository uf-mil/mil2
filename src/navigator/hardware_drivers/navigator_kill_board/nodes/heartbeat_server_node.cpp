#include <rclcpp/rclcpp.hpp>

#include "heartbeat_server.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create heartbeat server publishing to /network topic with 1.0 second period
    navigator_kill_board::HeartbeatServer server("/network", 1.0);

    RCLCPP_INFO(rclcpp::get_logger("heartbeat_server"), "Heartbeat server started, publishing to /network");

    rclcpp::spin(server.get_node());

    rclcpp::shutdown();
    return 0;
}
