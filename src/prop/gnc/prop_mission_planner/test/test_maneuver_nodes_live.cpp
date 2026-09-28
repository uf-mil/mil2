#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include "prop_maneuvers/geometry.hpp"
#include "prop_mission_planner/object_ref.hpp"
#include "tree_fixture.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// The maneuver-node tests that feed in a fake world. Kept apart from
// test_maneuver_nodes.cpp because the suite shares one Context: once odometry
// arrives the boat stays valid, which would break the no-odometry test there.
// Each test here releases the lock first and halts its tree last, so the
// shared TargetLock and motors start and end every test in the same state.

using prop_mission_planner::ObjectRef;

namespace
{

/// Adds a fake boat and a fake buoy, the two inputs a maneuver node waits for.
class ManeuverFixture : public TreeFixture
{
  protected:
    void SetUp() override
    {
        TreeFixture::SetUp();
        odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered/global", 10);
        markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", rclcpp::QoS(1));
    }

    /// Boat at the origin pointing along +x, buoy of radius 0.25 at (x, y).
    void publish_world(double x, double y)
    {
        publish_odometry();
        publish_buoy(x, y);
    }

    /// Boat at the origin pointing along +x.
    void publish_odometry()
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header.frame_id = "map";
        odometry.header.stamp = node_->now();
        odometry.pose.pose.orientation.w = 1.0;
        odometry_->publish(odometry);
    }

    /// A buoy of radius 0.25 at (x, y), stamped now.
    void publish_buoy(double x, double y)
    {
        visualization_msgs::msg::Marker box;
        box.header.frame_id = "map";
        box.header.stamp = node_->now();
        box.action = visualization_msgs::msg::Marker::ADD;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.pose.position.x = x;
        box.pose.position.y = y;
        box.pose.orientation.w = 1.0;
        box.scale.x = 0.5;
        box.scale.y = 0.5;
        box.scale.z = 1.0;
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(box);
        markers_->publish(markers);
    }

    /// Tick until `until` holds or 5 s pass, feeding the world (the buoy at
    /// buoy_x_, buoy_y_) each time.
    template <typename Predicate>
    BT::NodeStatus tick_until(BT::Tree &tree, Predicate until)
    {
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline)
        {
            publish_world(buoy_x_, buoy_y_);
            rclcpp::spin_some(node_);
            status = tree.tickOnce();
            if (until(status))
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return status;
    }

    static bool finished(BT::NodeStatus status)
    {
        return status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE;
    }

    /// Process incoming messages for `msec` milliseconds.
    void spin_for(int msec)
    {
        auto const deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(msec);
        while (std::chrono::steady_clock::now() < deadline)
        {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    /// Shrinks max_odometry_age for one test, so going stale takes a fraction
    /// of a second rather than the shipped 1 s, and puts it back afterwards
    /// even when an ASSERT ends the test early. The suite shares one Context.
    struct ShortOdometryAge
    {
        explicit ShortOdometryAge(double seconds) : saved(ctx_->settings->max_odometry_age_)
        {
            ctx_->settings->max_odometry_age_ = seconds;
        }
        ~ShortOdometryAge()
        {
            ctx_->settings->max_odometry_age_ = saved;
        }
        double saved;
    };

    double buoy_x_{ -4.0 };
    double buoy_y_{ -4.0 };
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
};

TEST_F(ManeuverFixture, LocksOnAndReportsWhatItLockedOnto)
{
    ctx_->maneuvers->lock.release();
    auto tree = build("FaceLocks", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <FaceObject target="{b}" ref="{locked}"/>
    </Sequence>)");

    tick_until(tree,
               [this](BT::NodeStatus)
               {
                   ObjectRef ignored;
                   return blackboard_->get("locked", ignored);
               });

    ObjectRef locked;
    ASSERT_TRUE(blackboard_->get("locked", locked)) << "never locked on";
    EXPECT_NEAR(locked.point.x, -4.0, 1e-6);
    EXPECT_NEAR(locked.point.y, -4.0, 1e-6);
    tree.haltTree();
}

TEST_F(ManeuverFixture, HaltingStopsTheMotors)
{
    ctx_->maneuvers->lock.release();
    std::optional<nav_msgs::msg::Path> last_plan;
    std::optional<geometry_msgs::msg::Twist> last_command;
    auto plans = node_->create_subscription<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).transient_local(),
                                                                 [&](nav_msgs::msg::Path const &m) { last_plan = m; });
    auto commands = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [&](geometry_msgs::msg::Twist const &m) { last_command = m; });

    auto tree = build("FaceHalted", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <FaceObject target="{b}"/>
    </Sequence>)");
    ASSERT_EQ(tick_until(tree, [](BT::NodeStatus s) { return s == BT::NodeStatus::RUNNING; }), BT::NodeStatus::RUNNING);

    // Seed a NON-empty plan, so an empty one afterwards proves the halt sent
    // it. Seeded after the node is running because FaceObject itself releases
    // guidance on its first step, which would empty the plan on its own.
    ctx_->maneuvers->driver.go_to(prop_maneuvers::Point{ 1.0, 1.0 });
    auto const seeded = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < seeded && !(last_plan && !last_plan->poses.empty()))
    {
        spin_for(20);
    }
    ASSERT_TRUE(last_plan && !last_plan->poses.empty()) << "the seeded plan never arrived";

    // Delivery is asynchronous: the turn command from the last tick can still
    // be in flight. Drain it first so the reset below means "since the halt".
    spin_for(200);
    last_plan.reset();
    last_command.reset();
    tree.haltTree();

    auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline && !(last_plan && last_command))
    {
        spin_for(20);
    }
    // Then keep listening a little: one publisher's messages arrive in order,
    // so anything stale still trailing in lands BEFORE the halt's stop, and
    // the last message kept is the stop itself.
    spin_for(200);
    ASSERT_TRUE(last_plan.has_value()) << "no plan published on halt";
    EXPECT_TRUE(last_plan->poses.empty());
    ASSERT_TRUE(last_command.has_value()) << "no cmd_vel published on halt";
    EXPECT_EQ(last_command->linear.x, 0.0);
    EXPECT_EQ(last_command->angular.z, 0.0);
    EXPECT_EQ(ctx_->active_maneuver, nullptr);
}

TEST_F(ManeuverFixture, FaceSucceedsWhenAlreadyFacing)
{
    ctx_->maneuvers->lock.release();
    // Boat at the origin pointing along +x and not moving; buoy dead ahead.
    buoy_x_ = 6.0;
    buoy_y_ = 0.0;
    auto tree = build("FaceAlreadyFacing", R"(<Sequence>
        <StaticObject point="6;0" ref="{b}"/>
        <FaceObject target="{b}"/>
    </Sequence>)");

    EXPECT_EQ(tick_until(tree, finished), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(ctx_->active_maneuver, nullptr) << "a finished maneuver still claims the boat";
    tree.haltTree();
}

TEST_F(ManeuverFixture, CircleWithABadBlackboardDirectionFails)
{
    ctx_->maneuvers->lock.release();
    blackboard_->set("d", std::string("cw"));
    auto tree = build("CircleBadBlackboardDirection", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <CircleObject target="{b}" direction="{d}"/>
    </Sequence>)");

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    EXPECT_NO_THROW(status = tick_until(tree, finished));
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(ctx_->active_maneuver, nullptr);
    tree.haltTree();
}

TEST_F(ManeuverFixture, ASecondManeuverCannotDriveWhileOneIsRunning)
{
    ctx_->maneuvers->lock.release();
    auto tree = build("TwoManeuversAtOnce", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <Parallel success_count="1" failure_count="1">
            <FaceObject name="first" target="{b}" ref="{first}"/>
            <FaceObject name="second" target="{b}" ref="{second}"/>
        </Parallel>
    </Sequence>)");

    EXPECT_EQ(tick_until(tree, finished), BT::NodeStatus::FAILURE);
    // Whichever locked on first drove; the other was refused before it could
    // build a maneuver (so it never reported a lock).
    ObjectRef ignored;
    bool const first = blackboard_->get("first", ignored);
    bool const second = blackboard_->get("second", ignored);
    EXPECT_NE(first, second) << "first locked: " << first << ", second locked: " << second;
    // The Parallel halted the one that was driving, which gave up its claim.
    EXPECT_EQ(ctx_->active_maneuver, nullptr);
    tree.haltTree();
}

// The EKF stalls mid-maneuver. The maneuvers steer off the last estimate and
// never ask its age, so without this the boat would keep driving on a frozen
// picture of itself. The node must stop it and fail.
TEST_F(ManeuverFixture, StaleOdometryStopsARunningManeuver)
{
    ctx_->maneuvers->lock.release();
    ShortOdometryAge const short_age(0.3);
    // (-4, -4) is behind and to the right: FaceObject is turning, so RUNNING.
    buoy_x_ = -4.0;
    buoy_y_ = -4.0;
    auto tree = build("FaceStaleOdometry", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <FaceObject target="{b}"/>
    </Sequence>)");
    ASSERT_EQ(tick_until(tree, [this](BT::NodeStatus) { return ctx_->active_maneuver != nullptr; }),
              BT::NodeStatus::RUNNING);

    // Clusters keep coming; odometry does not.
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline && status == BT::NodeStatus::RUNNING)
    {
        publish_buoy(buoy_x_, buoy_y_);
        rclcpp::spin_some(node_);
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(ctx_->active_maneuver, nullptr);
    EXPECT_FALSE(ctx_->maneuvers->lock.locked());
    tree.haltTree();
}

// A stale estimate counts as none when locking on: the buoy is in view, but
// placing it needs to know where the boat is NOW.
TEST_F(ManeuverFixture, StaleOdometryDoesNotLockOn)
{
    ctx_->maneuvers->lock.release();
    ShortOdometryAge const short_age(0.3);
    publish_odometry();
    spin_for(600);
    ASSERT_GT(ctx_->maneuvers->odometry_age(), 0.3);

    auto tree = build("FaceStaleOdometryNoLock", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <FaceObject target="{b}" lock_timeout="0.5" ref="{locked}"/>
    </Sequence>)");
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline && !finished(status))
    {
        publish_buoy(-4.0, -4.0);
        rclcpp::spin_some(node_);
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    ObjectRef ignored;
    EXPECT_FALSE(blackboard_->get("locked", ignored)) << "locked on against a stale position estimate";
    EXPECT_EQ(ctx_->active_maneuver, nullptr);
    tree.haltTree();
}

}  // namespace
