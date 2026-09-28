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
// rmw_zenoh_cpp (Jazzy) it needs `rmw_zenohd` running (CI starts it).
//
// The node, settings, lock and publisher are built ONCE for the whole suite,
// not per test: under rmw_zenoh_cpp (Jazzy) it stalls after several node
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
        // Shut the context down FIRST. lock_ owns a tf2_ros::TransformListener
        // with its own executor thread; under rmw_zenoh_cpp that thread's spin
        // can block until context shutdown wakes it, so destroying lock_ before
        // shutdown risks the reset() itself hanging while joining that thread.
        rclcpp::shutdown();
        lock_.reset();
        settings_.reset();
        publisher_.reset();
        node_.reset();
    }

    void SetUp() override
    {
        // The node, settings, lock and publisher persist across the whole
        // suite (see the class comment); only the lock's state needs
        // resetting between tests.
        lock_->release();
    }

    static visualization_msgs::msg::Marker box_at(double x, double y)
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
        return box;
    }

    // Publish a 0.5 m box at (x, y) until the lock reports it as a blob.
    void show_buoy_at(double x, double y)
    {
        auto box = box_at(x, y);
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

    // Publish a box at (x, y) stamped `age_seconds` in the past until the lock
    // has received it -- which, for a frame older than max_cluster_age, shows
    // only as blobs() naming the age in why().
    void show_old_buoy_at(double x, double y, double age_seconds)
    {
        auto box = box_at(x, y);
        auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline)
        {
            box.header.stamp = node_->now() - rclcpp::Duration::from_seconds(age_seconds);
            visualization_msgs::msg::MarkerArray markers;
            markers.markers.push_back(box);
            publisher_->publish(markers);
            rclcpp::spin_some(node_);
            (void)lock_->blobs();
            if (lock_->why().find("cluster frame is") != std::string::npos)
            {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        FAIL() << "the lock never reported the old frame at (" << x << ", " << y << ")";
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

// A mission runner chains maneuvers. If a failed acquire kept the previous
// lock, the maneuver for buoy 2 would find nothing and quietly drive to buoy 1.
TEST_F(TargetLockTest, FailedAcquireDropsThePreviousLock)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));

    // Nothing near (20, -3): the acquire fails...
    EXPECT_FALSE(lock_->acquire_near(Point{ 20.0, -3.0 }));
    // ...and the old buoy is no longer locked, with the reason kept.
    EXPECT_FALSE(lock_->locked());
    EXPECT_FALSE(lock_->why().empty());
}

TEST_F(TargetLockTest, FailedAcquireInFrontDropsThePreviousLock)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));

    // Boat at the origin facing +x: the buoy is behind it.
    EXPECT_FALSE(lock_->acquire_in_front(Point{ 0.0, 0.0 }, 0.0));
    EXPECT_FALSE(lock_->locked());
    EXPECT_EQ(lock_->why(), "nothing in front of the boat");
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

// A frame older than max_cluster_age (1 s) is not a picture of now. Before
// this, the lock kept the last frame for ever and, once its stamp had left the
// tf buffer, placed it with the boat's current pose: buoys that ride along
// with the boat. It must count as seeing nothing.
TEST_F(TargetLockTest, AnOldFrameShowsNothing)
{
    ASSERT_NO_FATAL_FAILURE(show_old_buoy_at(-4.0, -4.0, 5.0));

    EXPECT_TRUE(lock_->blobs().empty());
    EXPECT_FALSE(lock_->acquire_near(Point{ -4.0, -4.0 }));
    EXPECT_FALSE(lock_->locked());
    EXPECT_NE(lock_->why().find("cluster frame is"), std::string::npos) << lock_->why();
    EXPECT_NE(lock_->why().find("s old"), std::string::npos) << lock_->why();
    EXPECT_FALSE(lock_->acquire_in_front(Point{ 0.0, 0.0 }, std::atan2(-4.0, -4.0)));
    EXPECT_NE(lock_->why().find("cluster frame is"), std::string::npos) << lock_->why();
}

// ...and the age check only refuses OLD frames: once fresh ones arrive again
// the same buoy locks as usual.
TEST_F(TargetLockTest, AFreshFrameAfterAnOldOneLocks)
{
    ASSERT_NO_FATAL_FAILURE(show_old_buoy_at(-4.0, -4.0, 5.0));
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));

    EXPECT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));
    EXPECT_TRUE(lock_->why().empty()) << lock_->why();
}

// A locked target whose frames go old: refresh() fails and says why, keeping
// the remembered point (stale() ends the maneuver if nothing fresh comes).
TEST_F(TargetLockTest, RefreshOnAnOldFrameFailsAndSaysWhy)
{
    ASSERT_NO_FATAL_FAILURE(show_buoy_at(-4.0, -4.0));
    ASSERT_TRUE(lock_->acquire_near(Point{ -4.0, -4.0 }));

    ASSERT_NO_FATAL_FAILURE(show_old_buoy_at(-4.0, -4.0, 5.0));

    EXPECT_FALSE(lock_->refresh());
    EXPECT_NE(lock_->why().find("cluster frame is"), std::string::npos) << lock_->why();
    EXPECT_TRUE(lock_->locked());
}

}  // namespace
