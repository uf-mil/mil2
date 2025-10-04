#include "heartbeat_server.h"

namespace navigator_kill_board
{

HeartbeatServer::HeartbeatServer(std::string const& topic, double period) : period_(period)
{
    ros::NodeHandle nh;
    pub_ = nh.advertise<std_msgs::Header>(topic, 10);

    // Create timer that publishes at half the specified period
    timer_ = nh.createTimer(ros::Duration(period_ / 2.0), &HeartbeatServer::publish, this);
}

void HeartbeatServer::publish(ros::TimerEvent const& event)
{
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    pub_.publish(msg);
}

}  // namespace navigator_kill_board
