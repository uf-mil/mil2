#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "prop_maneuvers/geometry.hpp"

using namespace prop_maneuvers;

namespace
{
constexpr double kTol = 1e-9;
double deg(double d)
{
    return d * M_PI / 180.0;
}
}  // namespace

TEST(WrapAngle, LeavesSmallAnglesAlone)
{
    EXPECT_NEAR(wrap_angle(0.0), 0.0, kTol);
    EXPECT_NEAR(wrap_angle(0.5), 0.5, kTol);
    EXPECT_NEAR(wrap_angle(-0.5), -0.5, kTol);
}

TEST(WrapAngle, FoldsAnglesPastHalfTurn)
{
    EXPECT_NEAR(wrap_angle(3.0 * M_PI / 2.0), -M_PI / 2.0, kTol);
    EXPECT_NEAR(wrap_angle(-3.0 * M_PI / 2.0), M_PI / 2.0, kTol);
    EXPECT_NEAR(wrap_angle(2.0 * M_PI), 0.0, kTol);
}

TEST(Bearing, PointsTheRightWay)
{
    EXPECT_NEAR(bearing({ 0, 0 }, { 1, 0 }), 0.0, kTol);
    EXPECT_NEAR(bearing({ 0, 0 }, { 0, 1 }), M_PI / 2.0, kTol);
    EXPECT_NEAR(bearing({ 0, 0 }, { 0, -1 }), -M_PI / 2.0, kTol);
    EXPECT_NEAR(std::abs(bearing({ 0, 0 }, { -1, 0 })), M_PI, kTol);
}

TEST(Distance, IsStraightLine)
{
    EXPECT_NEAR(distance({ 0, 0 }, { 3, 4 }), 5.0, kTol);
    EXPECT_NEAR(distance({ 1, 1 }, { 1, 1 }), 0.0, kTol);
}

TEST(IsBlind, EmptyListSeesEverywhere)
{
    EXPECT_FALSE(is_blind(0.0, {}));
    EXPECT_FALSE(is_blind(deg(120.0), {}));
}

TEST(IsBlind, MirroredWedgesBehindTheBoat)
{
    // The guessed default: 90 to 160 on the left, and its mirror on the right.
    std::vector<BlindSpot> const spots{ { deg(90.0), deg(160.0) }, { deg(-160.0), deg(-90.0) } };

    EXPECT_FALSE(is_blind(deg(0.0), spots));    // straight ahead
    EXPECT_FALSE(is_blind(deg(45.0), spots));   // ahead and left
    EXPECT_TRUE(is_blind(deg(120.0), spots));   // behind the left shoulder
    EXPECT_TRUE(is_blind(deg(-120.0), spots));  // behind the right shoulder
    EXPECT_TRUE(is_blind(deg(90.0), spots));    // boundary counts as blind
    EXPECT_FALSE(is_blind(deg(180.0), spots));  // the gap directly behind
}

TEST(IsBlind, RangeThatWrapsAcrossDirectlyBehind)
{
    // A single wedge covering the back, written the only way it can be: from
    // 160 round through 180 to -160.
    std::vector<BlindSpot> const spots{ { deg(160.0), deg(-160.0) } };

    EXPECT_TRUE(is_blind(deg(180.0), spots));
    EXPECT_TRUE(is_blind(deg(170.0), spots));
    EXPECT_TRUE(is_blind(deg(-170.0), spots));
    EXPECT_FALSE(is_blind(deg(150.0), spots));
    EXPECT_FALSE(is_blind(deg(0.0), spots));
}

TEST(RingCorners, FirstCornerFacesTheBoatSoItDoesNotDoubleBack)
{
    // Boat out along +x; the first corner must also be out along +x.
    auto const corners = ring_corners({ 0, 0 }, { 10, 0 }, 5.0, 4, true);
    ASSERT_EQ(corners.size(), 4u);
    EXPECT_NEAR(corners[0].x, 5.0, kTol);
    EXPECT_NEAR(corners[0].y, 0.0, kTol);
}

TEST(RingCorners, CounterClockwiseGoesLeft)
{
    auto const corners = ring_corners({ 0, 0 }, { 10, 0 }, 5.0, 4, true);
    ASSERT_EQ(corners.size(), 4u);
    EXPECT_NEAR(corners[1].x, 0.0, kTol);
    EXPECT_NEAR(corners[1].y, 5.0, kTol);
    EXPECT_NEAR(corners[2].x, -5.0, kTol);
    EXPECT_NEAR(corners[3].y, -5.0, kTol);
}

TEST(RingCorners, ClockwiseGoesRight)
{
    auto const corners = ring_corners({ 0, 0 }, { 10, 0 }, 5.0, 4, false);
    ASSERT_EQ(corners.size(), 4u);
    EXPECT_NEAR(corners[1].x, 0.0, kTol);
    EXPECT_NEAR(corners[1].y, -5.0, kTol);
}

TEST(RingCorners, HonoursTheLegCountAndRejectsTooFew)
{
    EXPECT_EQ(ring_corners({ 0, 0 }, { 10, 0 }, 5.0, 6, true).size(), 6u);
    EXPECT_EQ(ring_corners({ 0, 0 }, { 10, 0 }, 5.0, 8, true).size(), 8u);
    EXPECT_TRUE(ring_corners({ 0, 0 }, { 10, 0 }, 5.0, 2, true).empty());
}

TEST(RingCorners, CentredOnSomewhereOtherThanTheOrigin)
{
    auto const corners = ring_corners({ 20, -3 }, { 30, -3 }, 6.0, 4, true);
    ASSERT_EQ(corners.size(), 4u);
    EXPECT_NEAR(corners[0].x, 26.0, kTol);
    EXPECT_NEAR(corners[0].y, -3.0, kTol);
    EXPECT_NEAR(corners[1].x, 20.0, kTol);
    EXPECT_NEAR(corners[1].y, 3.0, kTol);
}

TEST(StandoffPoint, StopsShortOfTheTarget)
{
    auto const p = standoff_point({ 0, 0 }, { 10, 0 }, 3.0);
    EXPECT_NEAR(p.x, 7.0, kTol);
    EXPECT_NEAR(p.y, 0.0, kTol);
}

TEST(StandoffPoint, DoesNotReverseWhenAlreadyCloseEnough)
{
    auto const p = standoff_point({ 0, 0 }, { 2, 0 }, 5.0);
    EXPECT_NEAR(p.x, 0.0, kTol);
    EXPECT_NEAR(p.y, 0.0, kTol);
}

TEST(DistanceToSegment, MeasuresAcrossTheLeg)
{
    auto const off = distance_to_segment({ 5, 2 }, { 0, 0 }, { 10, 0 });
    EXPECT_NEAR(off.perpendicular, 2.0, kTol);
    EXPECT_TRUE(off.within_segment);
}

TEST(DistanceToSegment, KnowsWhenThePointIsPastAnEnd)
{
    EXPECT_FALSE(distance_to_segment({ -5, 2 }, { 0, 0 }, { 10, 0 }).within_segment);
    EXPECT_FALSE(distance_to_segment({ 15, 2 }, { 0, 0 }, { 10, 0 }).within_segment);
}

TEST(DetourPoint, NothingWhenTheObstacleIsClearOfTheLeg)
{
    EXPECT_FALSE(detour_point({ 0, 0 }, { 10, 0 }, { 5, 5 }, 2.0).has_value());
}

TEST(DetourPoint, NothingWhenTheObstacleIsPastTheEnd)
{
    EXPECT_FALSE(detour_point({ 0, 0 }, { 10, 0 }, { 15, 0.5 }, 2.0).has_value());
}

TEST(DetourPoint, StepsAwayFromAnObstacleBesideTheLine)
{
    auto const p = detour_point({ 0, 0 }, { 10, 0 }, { 5, 0.5 }, 2.0);
    ASSERT_TRUE(p.has_value());
    // Pushed to the far side of the line from the obstacle, exactly keep_out away.
    EXPECT_NEAR(distance(*p, { 5, 0.5 }), 2.0, kTol);
    EXPECT_NEAR(p->x, 5.0, kTol);
    EXPECT_LT(p->y, 0.0);
}

TEST(DetourPoint, GoesLeftWhenTheObstacleSitsExactlyOnTheLine)
{
    auto const p = detour_point({ 0, 0 }, { 10, 0 }, { 5, 0 }, 2.0);
    ASSERT_TRUE(p.has_value());
    EXPECT_NEAR(p->x, 5.0, kTol);
    EXPECT_NEAR(p->y, 2.0, kTol);
}
