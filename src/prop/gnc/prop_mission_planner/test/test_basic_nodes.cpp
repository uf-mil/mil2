#include <gtest/gtest.h>

#include "prop_mission_planner/object_ref.hpp"
#include "tree_fixture.hpp"

using prop_mission_planner::ObjectRef;

TEST_F(TreeFixture, StaticObjectWritesTheReference)
{
    auto tree = build("StaticWrites", R"(<StaticObject point="-4;-4" ref="{b}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    auto const ref = blackboard_->get<ObjectRef>("b");
    EXPECT_DOUBLE_EQ(ref.point.x, -4.0);
    EXPECT_DOUBLE_EQ(ref.point.y, -4.0);
}

TEST_F(TreeFixture, StaticObjectRejectsABadPointAtLoad)
{
    EXPECT_ANY_THROW(build("StaticBadPoint", R"(<StaticObject point="4" ref="{b}"/>)"));
}

TEST_F(TreeFixture, StaticObjectNeedsAPointAndARef)
{
    EXPECT_ANY_THROW(build("StaticNoPoint", R"(<StaticObject ref="{b}"/>)"));
    EXPECT_ANY_THROW(build("StaticNoRef", R"(<StaticObject point="-4;-4"/>)"));
}

TEST_F(TreeFixture, RosTimeoutFailsWhenTheBudgetRunsOut)
{
    auto tree = build("TimeoutExpires", R"(<RosTimeout msec="0"><Sleep msec="60000"/></RosTimeout>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(TreeFixture, RosTimeoutPassesTheChildResultThrough)
{
    auto tree = build("TimeoutPasses", R"(<RosTimeout msec="60000"><AlwaysSuccess/></RosTimeout>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(TreeFixture, RosTimeoutNeedsMsec)
{
    EXPECT_ANY_THROW(build("TimeoutNoMsec", R"(<RosTimeout><AlwaysSuccess/></RosTimeout>)"));
}

TEST_F(TreeFixture, RosDelayWithZeroTicksTheChildAtOnce)
{
    auto tree = build("DelayZero", R"(<RosDelay delay_msec="0"><AlwaysSuccess/></RosDelay>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(TreeFixture, RosDelayWaits)
{
    auto tree = build("DelayWaits", R"(<RosDelay delay_msec="60000"><AlwaysSuccess/></RosDelay>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

// Nodes read the Context from the ROOT blackboard, so a subtree needs no
// ctx="{ctx}" remapping.
TEST_F(TreeFixture, NodesInsideASubtreeFindTheContext)
{
    auto &factory = prop_mission_planner::factory();
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
        <BehaviorTree ID="InnerTimeout"><RosTimeout msec="60000"><AlwaysSuccess/></RosTimeout></BehaviorTree>
        <BehaviorTree ID="OuterWithSubtree"><SubTree ID="InnerTimeout"/></BehaviorTree>
    </root>)");
    auto tree = factory.createTree("OuterWithSubtree", blackboard_);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
