#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "constants.h"
#include "heartbeat_server.h"
#include "simulated_kill_board.h"

class NavigatorKillBoardTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        // Initialize ROS2 if not already done
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override
    {
        // Don't shutdown ROS2 in TearDown as it may affect other tests
        // ROS2 will be shutdown in main()
    }
};

TEST_F(NavigatorKillBoardTest, NoopSerialTest)
{
    navigator_kill_board::NoopSerial noop;

    EXPECT_EQ(noop.port, "noop-serial");
    EXPECT_EQ(noop.in_waiting(), 0);
    EXPECT_EQ(noop.out_waiting(), 0);
    EXPECT_EQ(noop.read(10), "");
    EXPECT_EQ(noop.write("test"), 4);
}

TEST_F(NavigatorKillBoardTest, SimulatedSerialTest)
{
    navigator_kill_board::SimulatedSerial serial;

    // Test that buffer starts empty
    EXPECT_EQ(serial.in_waiting(), 0);

    // Test reading from empty buffer
    std::string result = serial.read(4);
    EXPECT_EQ(result, "");
    EXPECT_EQ(serial.in_waiting(), 0);

    // Test reset_input_buffer (should not crash)
    serial.reset_input_buffer();
    EXPECT_EQ(serial.in_waiting(), 0);
}

TEST_F(NavigatorKillBoardTest, SimulatedKillBoardBasicTest)
{
    navigator_kill_board::SimulatedKillBoard kill_board;

    EXPECT_EQ(kill_board.port, "simulated-kill-board");
    EXPECT_EQ(kill_board.in_waiting(), 0);

    // Test ping request
    std::string ping_request;
    ping_request += navigator_kill_board::PING_REQUEST;

    int bytes_written = kill_board.write(ping_request);
    EXPECT_EQ(bytes_written, 1);

    // Should have response in buffer
    EXPECT_GT(kill_board.in_waiting(), 0);
}

TEST_F(NavigatorKillBoardTest, ConstantsTest)
{
    // Test that constants are properly defined
    EXPECT_EQ(navigator_kill_board::TIMEOUT_SECONDS, 8.0);
    EXPECT_EQ(navigator_kill_board::PING_REQUEST, 0x20);
    EXPECT_EQ(navigator_kill_board::PING_RESPONSE, 0x30);

    // Test that KILLS vector contains expected values
    EXPECT_TRUE(std::find(navigator_kill_board::KILLS.begin(), navigator_kill_board::KILLS.end(), "OVERALL") !=
                navigator_kill_board::KILLS.end());
    EXPECT_TRUE(std::find(navigator_kill_board::KILLS.begin(), navigator_kill_board::KILLS.end(), "COMPUTER") !=
                navigator_kill_board::KILLS.end());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();

    // Shutdown ROS2
    rclcpp::shutdown();

    return result;
}
