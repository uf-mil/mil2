#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/context.hpp"

/// A ROS node, a mission Context and a blackboard holding it: enough to build
/// and tick trees without a simulator. Only tests that exchange messages
/// (publish or subscribe) need `rmw_zenohd` running (RMW is rmw_zenoh_cpp).
///
/// The node and Context are built ONCE for the whole test suite, not per test:
/// rmw_zenoh_cpp on this box leaks a participant's session threads when a node
/// is destroyed, and a dozen-plus create/destroy cycles in one process
/// reliably (if unpredictably) deadlocks a later node's construction for tens
/// of seconds to indefinitely. One node for the suite sidesteps the leak
/// instead of racing it.
class TreeFixture : public ::testing::Test
{
  protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
        // A private namespace: every topic the Context uses is relative, so
        // this suite's odometry, cluster_markers, plan and cmd_vel cannot reach
        // (or be reached by) a sim running on the same Zenoh router. Without
        // it, HaltingStopsTheMotors would publish to a live boat.
        node_ = std::make_shared<rclcpp::Node>("prop_mission_planner_test",
                                               "/prop_mission_planner_test_" + std::to_string(::getpid()));
        ctx_ = prop_mission_planner::make_context(node_);
    }
    static void TearDownTestSuite()
    {
        // Shut the context down FIRST. Context owns a TargetLock, whose
        // tf2_ros::TransformListener runs its own executor thread; under
        // rmw_zenoh_cpp that thread's spin can block until context shutdown
        // wakes it, so destroying ctx_ before shutdown risks the reset()
        // itself hanging while joining that thread.
        rclcpp::shutdown();
        ctx_.reset();
        node_.reset();
    }

    void SetUp() override
    {
        // Cheap and process-local: a fresh one per test, unlike the node and
        // Context above, so no test can see another's blackboard entries.
        blackboard_ = BT::Blackboard::create();
        blackboard_->set("ctx", ctx_);
    }

    /// Register one tree, `<BehaviorTree ID="id">body</BehaviorTree>`, and build
    /// it. Every test must use its own id: the factory is shared.
    BT::Tree build(std::string const &id, std::string const &body)
    {
        auto &factory = prop_mission_planner::factory();
        factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"" + id + "\">" + body +
                                             "</BehaviorTree></root>");
        return factory.createTree(id, blackboard_);
    }

    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<prop_mission_planner::Context> ctx_;
    BT::Blackboard::Ptr blackboard_;
};

inline rclcpp::Node::SharedPtr TreeFixture::node_;
inline std::shared_ptr<prop_mission_planner::Context> TreeFixture::ctx_;
