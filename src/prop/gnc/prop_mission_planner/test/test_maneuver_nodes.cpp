#include <gtest/gtest.h>

#include <exception>
#include <string>

#include "tree_fixture.hpp"

// The maneuver-node tests that never publish anything. The node and Context
// are shared by the whole suite (see TreeFixture), so a test that fed in
// odometry would leave a valid boat behind for NoPositionEstimate... below;
// the tests that need a world live in test_maneuver_nodes_live.cpp instead.

namespace
{

/// Loading `body` must throw, and for the reason expected: `needle` names the
/// bad port, so an unrelated load error cannot pass for it.
class ManeuverNodes : public TreeFixture
{
  protected:
    void expect_rejected(std::string const &id, std::string const &body, std::string const &needle)
    {
        try
        {
            build(id, body);
            ADD_FAILURE() << id << ": loaded, but should have been rejected";
        }
        catch (std::exception const &e)
        {
            EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << id << ": wrong error: " << e.what();
        }
    }
};

TEST_F(ManeuverNodes, CircleWithoutADirectionFailsToLoad)
{
    EXPECT_ANY_THROW(build("CircleNoDirection", R"(<CircleObject in_front="true"/>)"));
}

TEST_F(ManeuverNodes, CircleWithAnUnknownDirectionFailsToLoad)
{
    EXPECT_ANY_THROW(build("CircleBadDirection", R"(<CircleObject in_front="true" direction="cw"/>)"));
}

TEST_F(ManeuverNodes, TargetAndInFrontTogetherFailToLoad)
{
    EXPECT_ANY_THROW(build("FaceBoth", R"(<FaceObject target="{b}" in_front="true"/>)"));
}

TEST_F(ManeuverNodes, NeitherTargetNorInFrontFailsToLoad)
{
    EXPECT_ANY_THROW(build("FaceNeither", R"(<FaceObject/>)"));
}

TEST_F(ManeuverNodes, CircleWithANonPositiveRadiusFailsToLoad)
{
    expect_rejected("CircleZeroRadius", R"(<CircleObject in_front="true" direction="clockwise" radius="0"/>)",
                    "radius");
    expect_rejected("CircleNegativeRadius", R"(<CircleObject in_front="true" direction="clockwise" radius="-1"/>)",
                    "radius");
}

TEST_F(ManeuverNodes, CircleWithTooFewLegsFailsToLoad)
{
    expect_rejected("CircleTwoLegs", R"(<CircleObject in_front="true" direction="clockwise" legs="2"/>)", "legs");
}

TEST_F(ManeuverNodes, ApproachWithANegativeStandoffFailsToLoad)
{
    expect_rejected("ApproachNegativeStandoff", R"(<ApproachObject in_front="true" standoff="-0.5"/>)", "standoff");
}

TEST_F(ManeuverNodes, ANegativeLockTimeoutFailsToLoad)
{
    expect_rejected("FaceNegativeTimeout", R"(<FaceObject in_front="true" lock_timeout="-1"/>)", "lock_timeout");
}

TEST_F(ManeuverNodes, NoPositionEstimateFailsAtTheLockTimeout)
{
    auto tree = build("FaceNoOdometry", R"(<Sequence>
        <StaticObject point="-4;-4" ref="{b}"/>
        <FaceObject target="{b}" lock_timeout="0"/>
    </Sequence>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ManeuverNodes, AnUnsetTargetFails)
{
    auto tree = build("FaceUnsetTarget", R"(<FaceObject target="{nothing_here}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

}  // namespace
