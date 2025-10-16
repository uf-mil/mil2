// Copyright 2025 University of Florida Machine Intelligence Laboratory
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
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
    HeartbeatServer(std::string const &topic, double period);

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
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;  //!< ROS2 publisher
    double period_;                                            //!< Heartbeat period in seconds
    rclcpp::TimerBase::SharedPtr timer_;                       //!< Timer for periodic publishing
};

}  // namespace navigator_kill_board
