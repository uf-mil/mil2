#include "navigator_kill_board/heartbeat_server.h"

namespace navigator_kill_board
{

HeartbeatServer::HeartbeatServer(std::string const& topic, double period) : period_(period)
{
    node_ = rclcpp::Node::make_shared("heartbeat_server");
    pub_ = node_->create_publisher<std_msgs::msg::Header>(topic, 10);

    // Create timer that publishes at half the specified period
    auto timer_period = std::chrono::duration<double>(period_ / 2.0);
    timer_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                                      std::bind(&HeartbeatServer::publish, this));
}

void HeartbeatServer::publish()
{
    auto msg = std_msgs::msg::Header();
    msg.stamp = node_->now();
    pub_->publish(msg);
}

}  // namespace navigator_kill_board
