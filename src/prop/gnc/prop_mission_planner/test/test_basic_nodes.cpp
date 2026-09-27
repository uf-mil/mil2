#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "prop_mission_planner/object_ref.hpp"
#include "tree_fixture.hpp"

using prop_mission_planner::ObjectRef;

namespace
{

// Stays RUNNING forever and counts how often it is halted.
class RunsForever : public BT::StatefulActionNode
{
  public:
    using BT::StatefulActionNode::StatefulActionNode;
    static BT::PortsList providedPorts()
    {
        return {};
    }
    static int halted;
    BT::NodeStatus onStart() override
    {
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
        ++halted;
    }
};
int RunsForever::halted = 0;

/// Registers RunsForever into the shared factory exactly once, no matter how
/// many tests call this.
void ensure_runs_forever_registered()
{
    static bool const registered = []
    {
        prop_mission_planner::factory().registerNodeType<RunsForever>("RunsForever");
        return true;
    }();
    (void)registered;
}

}  // namespace

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
    EXPECT_ANY_THROW(build("StaticRefNotBlackboard", R"(<StaticObject point="-4;-4" ref="7"/>)"));
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

TEST_F(TreeFixture, RosTimeoutRejectsANegativeBudget)
{
    EXPECT_ANY_THROW(build("TimeoutNegative", R"(<RosTimeout msec="-5"><AlwaysSuccess/></RosTimeout>)"));
}

TEST_F(TreeFixture, RosTimeoutHaltsARunningChildWhenTimeRunsOut)
{
    ensure_runs_forever_registered();
    RunsForever::halted = 0;
    auto tree = build("TimeoutHaltsRunning", R"(<RosTimeout msec="50"><RunsForever/></RosTimeout>)");

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(RunsForever::halted, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(RunsForever::halted, 1);
}

TEST_F(TreeFixture, RosTimeoutRearmsForTheNextRun)
{
    ensure_runs_forever_registered();
    RunsForever::halted = 0;
    auto tree = build("TimeoutRearms", R"(<RosTimeout msec="50"><RunsForever/></RosTimeout>)");

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

    // A fresh tick re-arms the budget instead of staying expired forever.
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(TreeFixture, HaltingRosTimeoutHaltsTheChild)
{
    ensure_runs_forever_registered();
    RunsForever::halted = 0;
    auto tree = build("TimeoutHaltPropagates", R"(<RosTimeout msec="50"><RunsForever/></RosTimeout>)");

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    tree.haltTree();
    EXPECT_EQ(RunsForever::halted, 1);

    // The budget was disarmed by halt(), so this is a fresh run, not an
    // already-expired one.
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
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
