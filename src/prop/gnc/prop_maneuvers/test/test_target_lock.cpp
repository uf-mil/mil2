#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/target_lock.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

using prop_maneuvers::Point;

// Feeds the lock one clustered box in the map frame, the way pcd does. A box
// already in "map" needs no transform tree: tf2 answers map->map itself.
//
// The test publishes and subscribes over the real DDS graph, so under
// rmw_zenoh_cpp it needs `rmw_zenohd` running (see the plan's Conventions).
//
// The node, settings, lock and publisher are built ONCE for the whole suite,
// not per test: rmw_zenoh_cpp on this box stalls after several node
// create/destroy cycles in one process (see
// prop_mission_planner/test/tree_fixture.hpp for the same fix and a longer
// explanation), and TargetLock's tf2_ros::TransformListener spins up its own
// internal node and spin thread on top of the one passed in -- so a fresh
// node per test here would churn through two nodes per test, eight across
// this suite's four tests. Each test instead starts from a clean slate via
// lock_->release() in SetUp().
class TargetLockTest : public ::testing::Test
{
  protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
        // A private namespace per process, not just per test: "cluster_markers"
        // is the same relative topic name pcd publishes on in a live sim, so
        // without this the test's fake buoy would leak onto a running sim (or
        // the sim's real markers would leak into the test).
        node_ = std::make_shared<rclcpp::Node>("target_lock_test", "/target_lock_test_" + std::to_string(::getpid()));
        settings_ = std::make_unique<prop_maneuvers::Constants>(node_.get());
        lock_ = std::make_unique<prop_maneuvers::TargetLock>(node_.get(), *settings_);
        publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", rclcpp::QoS(1));
    }
    static void TearDownTestSuite()
    {
        lock_.reset();
        settings_.reset();
        publisher_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        // The node, settings, lock and publisher persist across the whole
        // suite (see the class comment); only the lock's state needs
        // resetting between tests.
        lock_->release();
    }

    // Publish a 0.5 m box at (x, y) until the lock reports it as a blob.
    void show_buoy_at(double x, double y)
    {
        visualization_msgs::msg::Marker box;
        box.header.frame_id = "map";
        box.action = visualization_msgs::msg::Marker::ADD;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.pose.position.x = x;
        box.pose.position.y = y;
        box.pose.orientation.w = 1.0;
        box.scale.x = 0.5;
        box.scale.y = 0.5;
        box.scale.z = 1.0;

        auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline)
        {
            box.header.stamp = node_->now();
            visualization_msgs::msg::MarkerArray markers;
            markers.markers.push_back(box);
            publisher_->publish(markers);
            rclcpp::spin_some(node_);
            for (auto const &blob : lock_->blobs())
            {
                if (std::abs(blob.centre.x - x) < 1e-6 && std::abs(blob.centre.y - y) < 1e-6)
                {
                    return;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        FAIL() << "the lock never received the box at (" << x << ", " << y << ")";
    }

    static rclcpp::Node::SharedPtr node_;
    static std::unique_ptr<prop_maneuvers::Constants> settings_;
    static std::unique_ptr<prop_maneuvers::TargetLock> lock_;
    static rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

rclcpp::Node::SharedPtr TargetLockTest::node_;
std::unique_ptr<prop_maneuvers::Constants> TargetLockTest::settings_;
std::unique_ptr<prop_maneuvers::TargetLock> TargetLockTest::lock_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr TargetLockTest::publisher_;

TEST_F(TargetLockTest, ReleaseDropsTheLock)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));
    ASSERT_TRUE(lock_->locked());

    lock_->release();

    EXPECT_FALSE(lock_->locked());
    EXPECT_TRUE(lock_->stale());
    EXPECT_EQ(lock_->radius(), 0.0);
    EXPECT_TRUE(lock_->why().empty());
}

// Why release() exists. The standalone programs lock once and exit, so this
// never mattered; a mission runner chains maneuvers, and without a release the
// maneuver for buoy 2 would drive to buoy 1.
TEST_F(TargetLockTest, FailedAcquireKeepsThePreviousLock)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));

    // Nothing near (20, -3): the acquire fails...
    EXPECT_FALSE(lock_->acquire_near(Point{ 20.0, -3.0 }));
    // ...and the old buoy is still locked.
    EXPECT_TRUE(lock_->locked());
    EXPECT_NEAR(lock_->point().x, -4.0, 1e-6);
    EXPECT_NEAR(lock_->point().y, -4.0, 1e-6);
    EXPECT_NEAR(lock_->radius(), 0.25, 1e-6);
}

TEST_F(TargetLockTest, ReleaseThenFailedAcquireLeavesNothingLocked)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));

    lock_->release();

    EXPECT_FALSE(lock_->acquire_near(Point{ 20.0, -3.0 }));
    EXPECT_FALSE(lock_->locked());
}

// The mission runner's normal path: release the last buoy, then lock onto the
// next one, rather than only ever failing to relock (as above).
TEST_F(TargetLockTest, ReleaseThenLocksOntoADifferentBuoy)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));

    lock_->release();

    ASSERT_NO_FATAL_FAILURE(show_buoy_at(4.0, 4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ 4.0, 4.0 }));

    EXPECT_NEAR(lock_->point().x, 4.0, 1e-6);
    EXPECT_NEAR(lock_->point().y, 4.0, 1e-6);
}

}  // namespace
