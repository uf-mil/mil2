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
#include <rclcpp/rclcpp.hpp>

#include "navigator_kill_board/heartbeat_server.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create heartbeat server publishing to /network topic with 1.0 second period
    navigator_kill_board::HeartbeatServer server("/network", 1.0);

    RCLCPP_INFO(rclcpp::get_logger("heartbeat_server"), "Heartbeat server started, publishing to /network");

    rclcpp::spin(server.get_node());

    rclcpp::shutdown();
    return 0;
}
