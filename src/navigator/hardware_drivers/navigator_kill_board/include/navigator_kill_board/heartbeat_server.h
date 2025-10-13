#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

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

    /**
     * @brief Get the ROS2 node pointer
     * @return Shared pointer to the ROS2 node
     */
    rclcpp::Node::SharedPtr get_node()
    {
        return node_;
    }

  private:
    /**
     * @brief Timer callback that publishes heartbeat messages
     */
    void publish();

    rclcpp::Node::SharedPtr node_;                             //!< ROS2 node pointer
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;  //!< ROS2 publisher for heartbeat messages
    double period_;                                            //!< Heartbeat period in seconds
    rclcpp::TimerBase::SharedPtr timer_;                       //!< Timer for periodic publishing
};

}  // namespace navigator_kill_board
