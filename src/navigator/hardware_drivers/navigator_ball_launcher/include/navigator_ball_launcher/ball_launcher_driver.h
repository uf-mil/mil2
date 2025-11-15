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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "navigator_ball_launcher/packets.h"

#include <navigator_msgs/srv/ball_launcher_drops.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace navigator_ball_launcher
{

/**
 * @brief ROS2 node that communicates with the ball launcher hardware device.
 *
 * Handles serial communication via electrical protocol, manages drop counter,
 * and responds to service requests for dropping balls and controlling spin.
 */
class BallLauncherDriver : public rclcpp::Node
{
  public:
    /**
     * @brief Initialize the ball launcher driver node.
     * @param port Serial port path (e.g., "/dev/ttyUSB0")
     * @param baudrate Serial communication speed (default 115200)
     */
    explicit BallLauncherDriver(std::string const& port, uint32_t baudrate = 115200);

    ~BallLauncherDriver() = default;

    /**
     * @brief Get the current number of balls dropped.
     * @return Drop count
     */
    int32_t get_drop_count() const;

  private:
    // Service handlers
    void drop_ball(std::shared_ptr<std_srvs::srv::Empty::Request> const request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void spin(std::shared_ptr<std_srvs::srv::SetBool::Request> const request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void number_of_drops(std::shared_ptr<navigator_msgs::srv::BallLauncherDrops::Request> const request,
                         std::shared_ptr<navigator_msgs::srv::BallLauncherDrops::Response> response);

    // Timer callbacks for response validation
    void check_for_dropped_ball(void);
    void check_for_spun(void);
    void check_for_valid_response(std::string const& event);
    void reset_ack(void);

    // ROS2 service servers
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr drop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr spin_service_;
    rclcpp::Service<navigator_msgs::srv::BallLauncherDrops>::SharedPtr drops_service_;

    // Response tracking
    bool heard_ack_{ false };
    bool heard_nack_{ false };
    int32_t drops_{ 0 };

    // Timer for checking responses
    rclcpp::TimerBase::SharedPtr temp_timer_;
};

}  // namespace navigator_ball_launcher
