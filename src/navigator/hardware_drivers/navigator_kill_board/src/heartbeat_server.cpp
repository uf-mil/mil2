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
#include "navigator_kill_board/heartbeat_server.h"

namespace navigator_kill_board
{

HeartbeatServer::HeartbeatServer(std::string const &topic, double period) : period_(period)
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
