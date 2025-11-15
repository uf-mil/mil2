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

#include "navigator_ball_launcher/ball_launcher_driver.h"

#include <chrono>

namespace navigator_ball_launcher
{

BallLauncherDriver::BallLauncherDriver(std::string const& port, uint32_t baudrate)
  : rclcpp::Node("ball_launcher_driver")
{
    (void)port;
    (void)baudrate;
    // TODO: Initialize serial connection to ball launcher hardware

    // Create service servers
    drop_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/drop_ball", [this](std::shared_ptr<std_srvs::srv::Empty::Request> const req,
                              std::shared_ptr<std_srvs::srv::Empty::Response> res) { this->drop_ball(req, res); });

    spin_service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/spin", [this](std::shared_ptr<std_srvs::srv::SetBool::Request> const req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> res) { this->spin(req, res); });

    drops_service_ = this->create_service<navigator_msgs::srv::BallLauncherDrops>(
        "~/number_of_drops", [this](std::shared_ptr<navigator_msgs::srv::BallLauncherDrops::Request> const req,
                                    std::shared_ptr<navigator_msgs::srv::BallLauncherDrops::Response> res)
        { this->number_of_drops(req, res); });

    RCLCPP_INFO(this->get_logger(), "Ball launcher device initialized on port %s at %u baud", port.c_str(), baudrate);
}

int32_t BallLauncherDriver::get_drop_count() const
{
    return drops_;
}

void BallLauncherDriver::drop_ball(std::shared_ptr<std_srvs::srv::Empty::Request> const request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;

    reset_ack();
    RCLCPP_INFO(this->get_logger(), "Dropping ball...");
    drops_++;
    // TODO: send_packet(ReleaseBallPacket());

    // Set up timer to check for response after 1 second
    temp_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { this->check_for_dropped_ball(); });
}

void BallLauncherDriver::spin(std::shared_ptr<std_srvs::srv::SetBool::Request> const request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    (void)response;

    reset_ack();
    if (request->data)
    {
        RCLCPP_INFO(this->get_logger(), "Spinning up the ball launcher...");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Stopping the ball launcher...");
    }
    // TODO: send_packet(SetSpinPacket(request->data));

    // Set up timer to check for response after 1 second
    temp_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { this->check_for_spun(); });
}

void BallLauncherDriver::number_of_drops(std::shared_ptr<navigator_msgs::srv::BallLauncherDrops::Request> const request,
                                         std::shared_ptr<navigator_msgs::srv::BallLauncherDrops::Response> response)
{
    (void)request;
    response->count = get_drop_count();
}

void BallLauncherDriver::reset_ack()
{
    heard_ack_ = false;
    heard_nack_ = false;
}

void BallLauncherDriver::check_for_valid_response(std::string const& event)
{
    // Cancel the timer
    if (temp_timer_)
    {
        temp_timer_.reset();
    }

    if (heard_nack_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to %s (heard NACK from board)", event.c_str());
    }
    else if (!heard_ack_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to %s (no response from board)", event.c_str());
    }
}

void BallLauncherDriver::check_for_dropped_ball()
{
    check_for_valid_response("drop ball");
}

void BallLauncherDriver::check_for_spun()
{
    check_for_valid_response("set spin");
}

}  // namespace navigator_ball_launcher
