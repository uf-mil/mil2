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

#include "navigator_kill_board/simulated_kill_board.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "Starting Navigator Simulated Kill Board");

    // Create the simulated kill board
    navigator_kill_board::SimulatedKillBoard kill_board;

    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "Simulated kill board initialized");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "Available services:");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_FRONT_PORT");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_AFT_PORT");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_FRONT_STARBOARD");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_AFT_STARBOARD");
    RCLCPP_INFO(rclcpp::get_logger("simulated_kill_board"), "  - BUTTON_REMOTE");

    rclcpp::spin(kill_board.get_node());

    rclcpp::shutdown();
    return 0;
}
