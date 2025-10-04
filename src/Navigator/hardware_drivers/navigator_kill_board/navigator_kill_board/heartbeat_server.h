#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <string>

namespace navigator_kill_board
{

/**
 * @brief Publishes heartbeat messages at regular intervals
 *
 * This class provides a simple heartbeat server that publishes Header messages
 * to a specified topic at half the specified period rate.
 */
class HeartbeatServer
{
  public:
    /**
     * @brief Construct a new Heartbeat Server
     *
     * @param topic The ROS topic to publish heartbeat messages to
     * @param period The period in seconds for heartbeat timing (publishes at half this rate)
     */
    HeartbeatServer(std::string const& topic, double period);

    /**
     * @brief Destructor
     */
    ~HeartbeatServer() = default;

  private:
    /**
     * @brief Timer callback that publishes heartbeat messages
     *
     * @param event Timer event (unused)
     */
    void publish(ros::TimerEvent const& event);

    ros::Publisher pub_;  //!< ROS publisher for heartbeat messages
    double period_;       //!< Heartbeat period in seconds
    ros::Timer timer_;    //!< Timer for periodic publishing
};

}  // namespace navigator_kill_board
