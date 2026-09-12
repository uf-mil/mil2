#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

TEST(IsBlind, ZeroWidthSpotBlindsNothing)
{
    // from == to: the code requires span > 0.0, so a degenerate spot never
    // matches, not even the angle it sits on.
    std::vector<BlindSpot> const spots{ { deg(45.0), deg(45.0) } };

    EXPECT_FALSE(is_blind(deg(45.0), spots));
    EXPECT_FALSE(is_blind(deg(0.0), spots));
    EXPECT_FALSE(is_blind(deg(180.0), spots));
}

TEST(IsBlind, SpotWiderThanAHalfTurn)
{
    // 0 to 270 degrees, going the long way round (counter-clockwise, i.e. the
    // positive-wrap direction from `from` to `to`).
    std::vector<BlindSpot> const spots{ { deg(0.0), deg(270.0) } };

    EXPECT_TRUE(is_blind(deg(269.0), spots));
    EXPECT_FALSE(is_blind(deg(271.0), spots));
}

TEST(IsBlind, TwoOverlappingSpotsOrTogether)
{
    // 0-100 and 50-150 overlap on 50-100; each angle should be blind if
    // EITHER spot covers it, not only where both do.
    std::vector<BlindSpot> const spots{ { deg(0.0), deg(100.0) }, { deg(50.0), deg(150.0) } };

    EXPECT_TRUE(is_blind(deg(10.0), spots));    // only the first spot
    EXPECT_TRUE(is_blind(deg(75.0), spots));    // both spots
    EXPECT_TRUE(is_blind(deg(140.0), spots));   // only the second spot
    EXPECT_FALSE(is_blind(deg(170.0), spots));  // neither
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

TEST(DistanceToSegment, ZeroLengthLegReturnsDistanceToThatPoint)
{
    // a == b: the "leg" is a single point. distance_to_segment falls back to
    // plain distance-to-a-point and reports within_segment == false, which
    // is what detour_point relies on to skip the length_squared == 0 case.
    auto const off = distance_to_segment({ 3, 4 }, { 0, 0 }, { 0, 0 });
    EXPECT_NEAR(off.perpendicular, 5.0, kTol);
    EXPECT_FALSE(off.within_segment);
}

// ── distance_to_polyline ─────────────────────────────────────────────────

TEST(DistanceToPolyline, ClampsToTheSegmentEndsUnlikeDistanceToSegment)
{
    // distance_to_segment reports the perpendicular to the INFINITE line, so
    // it calls this point 0.0 away. The boat never goes there: the leg stops
    // at (10, 0), so the real closest approach is 0.5 m.
    std::vector<Point> const path{ { 0, 0 }, { 10, 0 } };
    EXPECT_NEAR(distance_to_polyline({ 10.5, 0 }, path), 0.5, kTol);
    EXPECT_NEAR(distance_to_segment({ 10.5, 0 }, { 0, 0 }, { 10, 0 }).perpendicular, 0.0, kTol);
}

TEST(DistanceToPolyline, TakesTheSmallestOverEveryLeg)
{
    std::vector<Point> const path{ { 0, 0 }, { 10, 0 }, { 10, 10 } };
    // Nearest to the second leg, not the first.
    EXPECT_NEAR(distance_to_polyline({ 12, 5 }, path), 2.0, kTol);
}

TEST(DistanceToPolyline, HandlesADegeneratePath)
{
    EXPECT_NEAR(distance_to_polyline({ 3, 4 }, { { 0, 0 } }), 5.0, kTol);
    EXPECT_EQ(distance_to_polyline({ 3, 4 }, {}), std::numeric_limits<double>::infinity());
}

// ── plan_detour ──────────────────────────────────────────────────────────
//
// Shared setup for every case below: a 0.25 m buoy, a boat whose hull reaches
// 0.5 m either side of base_link, a 0.15 m trigger and 1.0 m of swing. That
// puts the trigger at 0.25 + 0.5 + 0.15 = 0.90 m from the buoy centre and the
// swing at 0.25 + 0.5 + 1.0 = 1.75 m.

namespace
{
constexpr double kHalfWidth{ 0.5 };
constexpr double kMinGap{ 0.15 };
constexpr double kClearance{ 1.0 };
Blob buoy(double x, double y)
{
    return Blob{ { x, y }, 0.25 };
}
}  // namespace

TEST(PlanDetour, NothingWhenTheObstacleIsClearOfTheLeg)
{
    auto const d = plan_detour({ 0, 0 }, { 10, 0 }, buoy(5, 2.0), kHalfWidth, kMinGap, kClearance);
    EXPECT_EQ(d.need, DetourNeed::None);
}

TEST(PlanDetour, NothingWhenTheObstacleIsPastTheEnd)
{
    auto const d = plan_detour({ 0, 0 }, { 10, 0 }, buoy(12, 0.1), kHalfWidth, kMinGap, kClearance);
    EXPECT_EQ(d.need, DetourNeed::None);
}

TEST(PlanDetour, StraddlesABlockingObstacleAndDeliversTheClearance)
{
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(10, 0.3), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);

    // One waypoint before the buoy and one after, both pushed to the far side.
    EXPECT_NEAR(d.before.x, 8.25, kTol);
    EXPECT_NEAR(d.before.y, -1.45, kTol);
    EXPECT_NEAR(d.after.x, 11.75, kTol);
    EXPECT_NEAR(d.after.y, -1.45, kTol);

    // The point of the whole exercise: measure the PATH, not the waypoints.
    EXPECT_NEAR(d.achieved, kClearance, kTol);
}

TEST(PlanDetour, TheDrivenPathIsWhatIsMeasuredNotTheWaypoints)
{
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(10, 0.3), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);

    // Recomputed here independently of plan_detour's own bookkeeping.
    std::vector<Point> const driven{ { 0, 0 }, d.before, d.after, { 20, 0 } };
    double const hull_gap = distance_to_polyline({ 10, 0.3 }, driven) - 0.25 - kHalfWidth;
    EXPECT_NEAR(hull_gap, kClearance, kTol);
}

TEST(PlanDetour, RefusesToSwingWhenTheBoatStartsTooClose)
{
    // 1.51 m from the buoy centre, inside the 1.75 m the swing needs.
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(1.5, 0.2), kHalfWidth, kMinGap, kClearance);
    EXPECT_EQ(d.need, DetourNeed::TooCloseToSwing);
}

TEST(PlanDetour, BestEffortWhenTheGoalItselfSitsInsideTheClearance)
{
    // The path ends at the goal, so a buoy 0.58 m from it caps what any route
    // can achieve. Report the shortfall; do not refuse -- on a buoy course
    // this is ordinary, and the caller chose the destination.
    auto const d = plan_detour({ 0, 0 }, { 10, 0 }, buoy(9.5, 0.3), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);
    EXPECT_LT(d.achieved, kClearance);
}

TEST(PlanDetour, GoesLeftWhenTheObstacleSitsExactlyOnTheLine)
{
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(5, 0), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);
    EXPECT_NEAR(d.before.x, 3.25, kTol);
    EXPECT_NEAR(d.before.y, 1.75, kTol);
    EXPECT_NEAR(d.after.x, 6.75, kTol);
    EXPECT_NEAR(d.after.y, 1.75, kTol);
}

TEST(PlanDetour, SpacingEqualsTheSidewaysOffset)
{
    // The swept result: any narrower under-delivers, any wider only drives
    // further. Recorded as a test so a future "tidy-up" cannot quietly change it.
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(10, 0.3), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);
    double const spacing = distance(d.before, d.after) / 2.0;
    double const offset = 0.25 + kHalfWidth + kClearance;
    EXPECT_NEAR(spacing, offset, kTol);
}

TEST(PlanDetour, NeverPutsTheFirstWaypointBehindTheBoat)
{
    // An obstacle close on the bow passes both gates: 0.85 m off the leg trips
    // the 0.90 m trigger, and at 1.81 m away the boat is outside the 1.75 m
    // swing, so this is not TooCloseToSwing. Unclamped, `before` lands at
    // along-track -0.15 m -- astern. guidance restarts at waypoint 0 on every
    // hand-over and scales forward speed by the cosine of the bearing error,
    // so a point astern means the boat spins on the spot instead of driving.
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(1.6, 0.85), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);
    EXPECT_GE(d.before.x, -kTol) << "first straddle waypoint is behind the boat";
    EXPECT_GE(d.after.x, -kTol);
}

TEST(PlanDetour, NeverPutsTheSecondWaypointPastTheGoal)
{
    // An obstacle close to the goal pushes `after` beyond it, which would
    // carry the bow nearer the object than the standoff promises before the
    // boat turned back for the goal.
    auto const d = plan_detour({ 0, 0 }, { 10, 0 }, buoy(9.5, 0.3), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);
    EXPECT_LE(d.after.x, 10.0 + kTol) << "second straddle waypoint is past the goal";
    EXPECT_LE(d.before.x, 10.0 + kTol);
}

TEST(PlanDetour, ClampingStillReportsWhatThePathActuallyAchieves)
{
    // A clamped straddle delivers less than was asked for. It must still
    // measure the path it really produced, so the caller's under-delivery
    // warning stays truthful rather than quoting an ideal it did not drive.
    auto const d = plan_detour({ 0, 0 }, { 20, 0 }, buoy(1.6, 0.85), kHalfWidth, kMinGap, kClearance);
    ASSERT_EQ(d.need, DetourNeed::Straddle);

    std::vector<Point> const driven{ { 0, 0 }, d.before, d.after, { 20, 0 } };
    double const hull_gap = distance_to_polyline({ 1.6, 0.85 }, driven) - 0.25 - kHalfWidth;
    EXPECT_NEAR(d.achieved, hull_gap, kTol);
}

TEST(MatchNearest, PicksTheClosestBlob)
{
    std::vector<Blob> const blobs{ { { 10.0, 0.0 }, 0.25 }, { { 0.5, 0.0 }, 0.25 } };
    auto const match = match_nearest(blobs, { 0.0, 0.0 }, 3.0, 1.0);
    ASSERT_TRUE(match.ok);
    EXPECT_NEAR(match.blob.centre.x, 0.5, kTol);
}

TEST(MatchNearest, FailsWhenThereAreNoBlobs)
{
    auto const match = match_nearest({}, { 0.0, 0.0 }, 3.0, 1.0);
    EXPECT_FALSE(match.ok);
    EXPECT_EQ(match.failure, MatchFailure::NoBlobs);
}

TEST(MatchNearest, FailsWhenTheNearestIsTooFarFromThePrediction)
{
    std::vector<Blob> const blobs{ { { 10.0, 0.0 }, 0.25 } };
    auto const match = match_nearest(blobs, { 0.0, 0.0 }, 3.0, 1.0);
    EXPECT_FALSE(match.ok);
    EXPECT_EQ(match.failure, MatchFailure::TooFar);
}

TEST(MatchNearest, RefusesToGuessBetweenTwoEquallyPlausibleBlobs)
{
    // Two buoys side by side, both about a metre from where we expected one.
    std::vector<Blob> const blobs{ { { 1.0, 0.0 }, 0.25 }, { { -1.0, 0.0 }, 0.25 } };
    auto const match = match_nearest(blobs, { 0.0, 0.0 }, 3.0, 1.0);
    EXPECT_FALSE(match.ok);
    EXPECT_EQ(match.failure, MatchFailure::Ambiguous);
}

TEST(MatchNearest, AcceptsWhenTheRunnerUpIsClearlyFurtherAway)
{
    std::vector<Blob> const blobs{ { { 0.2, 0.0 }, 0.25 }, { { 2.8, 0.0 }, 0.25 } };
    auto const match = match_nearest(blobs, { 0.0, 0.0 }, 3.0, 1.0);
    ASSERT_TRUE(match.ok);
    EXPECT_NEAR(match.blob.centre.x, 0.2, kTol);
}

TEST(MatchNearest, PinsBehaviorAtExactlyMatchRadius)
{
    // The nearest blob sits exactly on the match_radius boundary. The
    // implementation only rejects when the range is strictly greater than
    // match_radius, so this is a deliberate accept, not an accident of
    // floating point -- pinned here so nobody "fixes" it to `>=` later.
    std::vector<Blob> const blobs{ { { 3.0, 0.0 }, 0.25 } };
    auto const match = match_nearest(blobs, { 0.0, 0.0 }, 3.0, 1.0);
    ASSERT_TRUE(match.ok);
    EXPECT_NEAR(match.blob.centre.x, 3.0, kTol);
}

TEST(MatchNearest, PinsBehaviorWhenTheGapIsExactlyTheAmbiguousMargin)
{
    // The runner-up is exactly ambiguous_margin further away than the
    // nearest. The implementation only flags Ambiguous when the gap is
    // strictly less than ambiguous_margin, so a gap equal to the margin is a
    // deliberate accept -- pinned here so nobody "fixes" it to `<=` later.
    std::vector<Blob> const blobs{ { { 0.2, 0.0 }, 0.25 }, { { 1.2, 0.0 }, 0.25 } };
    auto const match = match_nearest(blobs, { 0.0, 0.0 }, 3.0, 1.0);
    ASSERT_TRUE(match.ok);
    EXPECT_NEAR(match.blob.centre.x, 0.2, kTol);
}

// ── clear_behind ─────────────────────────────────────────────────────────
//
// Boat at the origin facing +x throughout unless stated. Hull reaches 0.5 m
// either side and 1.0 m behind base_link, and the reverse is 2.0 m, so the
// swept strip runs from base_link to 3.0 m behind it, 0.5 m either side.

TEST(ClearBehind, ClearWhenThereAreNoBlobs)
{
    EXPECT_TRUE(clear_behind({}, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
}

TEST(ClearBehind, NotClearWhenSomethingSitsInTheStrip)
{
    std::vector<Blob> const blobs{ { { -2.0, 0.0 }, 0.25 } };
    EXPECT_FALSE(clear_behind(blobs, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
}

TEST(ClearBehind, ClearWhenTheBlobIsBeyondTheSweptDistance)
{
    // Strip ends 3.0 m back; this blob's near edge is at 3.25 m.
    std::vector<Blob> const blobs{ { { -3.5, 0.0 }, 0.25 } };
    EXPECT_TRUE(clear_behind(blobs, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
}

TEST(ClearBehind, ClearWhenTheBlobIsBesideTheStrip)
{
    // 2.0 m to the side; strip edge is at 0.5 m, blob edge at 1.75 m.
    std::vector<Blob> const blobs{ { { -2.0, 2.0 }, 0.25 } };
    EXPECT_TRUE(clear_behind(blobs, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
}

TEST(ClearBehind, NotClearWhenTheBlobOverlapsTheStripEdge)
{
    // Centre 0.7 m to the side, radius 0.25, so it reaches 0.45 m -- inside
    // the 0.5 m half-width.
    std::vector<Blob> const blobs{ { { -2.0, 0.7 }, 0.25 } };
    EXPECT_FALSE(clear_behind(blobs, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
}

TEST(ClearBehind, ClearWhenTheBlobIsInFront)
{
    std::vector<Blob> const blobs{ { { 5.0, 0.0 }, 0.25 } };
    EXPECT_TRUE(clear_behind(blobs, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
}

TEST(ClearBehind, TheStripFollowsTheBoatNotTheMap)
{
    // Same blob, twice. Facing +x it is in front and irrelevant; facing +y
    // (90 degrees) "behind" points down the -y axis, so it is still clear --
    // but facing -x (180 degrees) puts it squarely behind.
    std::vector<Blob> const blobs{ { { 2.0, 0.0 }, 0.25 } };
    EXPECT_TRUE(clear_behind(blobs, { 0, 0 }, 0.0, 2.0, 0.5, 1.0));
    EXPECT_TRUE(clear_behind(blobs, { 0, 0 }, M_PI / 2.0, 2.0, 0.5, 1.0));
    EXPECT_FALSE(clear_behind(blobs, { 0, 0 }, M_PI, 2.0, 0.5, 1.0));
}

// ── along_track and obstacle_ahead ───────────────────────────────────────
//
// The pair behind "have I got past that yet?". A committed straddle has to be
// held until the obstacle is behind the boat, because a plan made half way
// along it no longer sees the obstacle as blocking and would hand back the
// straight line the straddle exists to avoid.

TEST(AlongTrack, MeasuresForwardAndBackwardAlongTheHeading)
{
    EXPECT_NEAR(along_track({ 0, 0 }, 0.0, { 5, 0 }), 5.0, kTol);
    EXPECT_NEAR(along_track({ 0, 0 }, 0.0, { -5, 0 }), -5.0, kTol);
}

TEST(AlongTrack, IgnoresSidewaysOffsetEntirely)
{
    // Level with the boat but a long way to the side is still level with it.
    EXPECT_NEAR(along_track({ 0, 0 }, 0.0, { 5, 100 }), 5.0, kTol);
    EXPECT_NEAR(along_track({ 0, 0 }, 0.0, { 0, 100 }), 0.0, kTol);
}

TEST(AlongTrack, FollowsTheHeadingNotTheMapAxes)
{
    EXPECT_NEAR(along_track({ 0, 0 }, M_PI / 2.0, { 0, 5 }), 5.0, kTol);
    EXPECT_NEAR(along_track({ 0, 0 }, M_PI, { -5, 0 }), 5.0, kTol);
}

TEST(ObstacleAhead, TrueWhileTheObstacleIsStillInFront)
{
    EXPECT_TRUE(obstacle_ahead({ 0, 0 }, 0.0, Blob{ { 5, 0 }, 0.25 }, 1.0));
}

TEST(ObstacleAhead, StillAheadWhenLevelWithTheBoat)
{
    // base_link is level with it, but the hull reaches 1 m further back and is
    // still alongside. Letting go here would cut the corner on the way out.
    EXPECT_TRUE(obstacle_ahead({ 0, 0 }, 0.0, Blob{ { 0, 2 }, 0.25 }, 1.0));
}

TEST(ObstacleAhead, BehindOnlyOnceItsSurfaceClearsTheBackOfTheHull)
{
    // Threshold is -(radius + hull_behind) = -1.25 m.
    EXPECT_TRUE(obstacle_ahead({ 0, 0 }, 0.0, Blob{ { -1.2, 0 }, 0.25 }, 1.0));
    EXPECT_FALSE(obstacle_ahead({ 0, 0 }, 0.0, Blob{ { -1.3, 0 }, 0.25 }, 1.0));
}

TEST(ObstacleAhead, ABiggerObstacleStaysAheadForLonger)
{
    // Same centre, same hull: only the radius differs, and the wide one is
    // still alongside when the narrow one has been cleared.
    EXPECT_FALSE(obstacle_ahead({ 0, 0 }, 0.0, Blob{ { -1.5, 0 }, 0.25 }, 1.0));
    EXPECT_TRUE(obstacle_ahead({ 0, 0 }, 0.0, Blob{ { -1.5, 0 }, 1.00 }, 1.0));
}
