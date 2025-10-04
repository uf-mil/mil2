#include <gtest/gtest.h>
#include <ros/ros.h>

#include "navigator_kill_board/constants.h"
#include "navigator_kill_board/heartbeat_server.h"
#include "navigator_kill_board/simulated_kill_board.h"

class NavigatorKillBoardTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        // Initialize ROS if not already done
        if (!ros::isInitialized())
        {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_navigator_kill_board");
        }
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

    EXPECT_EQ(serial.in_waiting(), 0);

    // Test that buffer works correctly
    serial.buffer_ = "test_data";
    EXPECT_EQ(serial.in_waiting(), 9);

    std::string result = serial.read(4);
    EXPECT_EQ(result, "test");
    EXPECT_EQ(serial.in_waiting(), 5);

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
    return RUN_ALL_TESTS();
}
