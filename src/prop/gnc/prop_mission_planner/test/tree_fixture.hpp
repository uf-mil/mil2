#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/context.hpp"

/// A ROS node, a mission Context and a blackboard holding it: enough to build
/// and tick trees without a simulator. Tests that publish or subscribe need
/// `rmw_zenohd` running (RMW is rmw_zenoh_cpp).
class TreeFixture : public ::testing::Test
{
  protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        // A private namespace: every topic the Context uses is relative, so
        // this test's odometry, cluster_markers, plan and cmd_vel cannot reach
        // (or be reached by) a sim running on the same Zenoh router. Without
        // it, HaltingStopsTheMotors would publish to a live boat.
        node_ = std::make_shared<rclcpp::Node>("prop_mission_planner_test",
                                               "/prop_mission_planner_test_" + std::to_string(::getpid()));
        ctx_ = prop_mission_planner::make_context(node_);
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

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<prop_mission_planner::Context> ctx_;
    BT::Blackboard::Ptr blackboard_;
};
