#include "prop_planner/geometry.hpp"

#include <gtest/gtest.h>

using prop_planner::Capsule;
using prop_planner::contains;
using prop_planner::distance_to_segment;
using prop_planner::hits;
using prop_planner::inflate;
using prop_planner::length;
using prop_planner::Point;
using prop_planner::segment_distance;
using prop_planner::segments_cross;

TEST(Geometry, DistanceToSegmentClampsToTheEndpoints)
{
    // Beside the middle of the segment.
    EXPECT_NEAR(distance_to_segment({ 5.0, 3.0 }, { 0.0, 0.0 }, { 10.0, 0.0 }), 3.0, 1e-9);
    // Past the end, so the nearest point is the endpoint itself.
    EXPECT_NEAR(distance_to_segment({ 14.0, 0.0 }, { 0.0, 0.0 }, { 10.0, 0.0 }), 4.0, 1e-9);
    EXPECT_NEAR(distance_to_segment({ -3.0, 4.0 }, { 0.0, 0.0 }, { 10.0, 0.0 }), 5.0, 1e-9);
}

TEST(Geometry, DistanceToASegmentOfNoLengthIsDistanceToThePoint)
{
    EXPECT_NEAR(distance_to_segment({ 3.0, 4.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }), 5.0, 1e-9);
}

TEST(Geometry, CrossingSegmentsAreDetected)
{
    EXPECT_TRUE(segments_cross({ -1.0, 0.0 }, { 1.0, 0.0 }, { 0.0, -1.0 }, { 0.0, 1.0 }));
    EXPECT_FALSE(segments_cross({ -1.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 }, { 0.0, 2.0 }));
    EXPECT_FALSE(segments_cross({ 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 }, { 1.0, 1.0 }));
}

TEST(Geometry, SegmentDistanceIsZeroWhereTheyCross)
{
    EXPECT_NEAR(segment_distance({ -1.0, 0.0 }, { 1.0, 0.0 }, { 0.0, -1.0 }, { 0.0, 1.0 }), 0.0, 1e-9);
}

TEST(Geometry, SegmentDistanceHandlesParallelAndSkewPairs)
{
    // Parallel, two apart.
    EXPECT_NEAR(segment_distance({ 0.0, 0.0 }, { 10.0, 0.0 }, { 0.0, 2.0 }, { 10.0, 2.0 }), 2.0, 1e-9);
    // End to end along the same line.
    EXPECT_NEAR(segment_distance({ 0.0, 0.0 }, { 10.0, 0.0 }, { 13.0, 0.0 }, { 20.0, 0.0 }), 3.0, 1e-9);
    // Perpendicular but not reaching.
    EXPECT_NEAR(segment_distance({ 0.0, 0.0 }, { 10.0, 0.0 }, { 5.0, 4.0 }, { 5.0, 9.0 }), 4.0, 1e-9);
}

// The property the whole shape rests on: a long obstacle stays slim, where a
// circle enclosing it would have to be as wide as it is long.
TEST(Geometry, ACapsuleIsSlimWhereACircleWouldBeFat)
{
    Capsule const dock{ { -9.0, 0.0 }, { 9.0, 0.0 }, 1.0 };

    // Abeam the dock and well clear of it.
    EXPECT_FALSE(hits(dock, { -20.0, 6.0 }, { 20.0, 6.0 }));
    // A circle circumscribing the same 20 x 2 footprint would have a radius of
    // just over 10 m and would have blocked that path.
    EXPECT_LT(6.0, 0.5 * std::hypot(20.0, 2.0));

    // Straight through it is still blocked.
    EXPECT_TRUE(hits(dock, { -20.0, 0.0 }, { 20.0, 0.0 }));
}

TEST(Geometry, InflationOnlyChangesTheRadius)
{
    Capsule const grown = inflate(Capsule{ { 0.0, 0.0 }, { 5.0, 0.0 }, 1.0 }, 2.0);

    EXPECT_NEAR(grown.radius, 3.0, 1e-9);
    EXPECT_NEAR(length(grown), 5.0, 1e-9);
    EXPECT_NEAR(grown.a.x, 0.0, 1e-9);
    EXPECT_NEAR(grown.b.x, 5.0, 1e-9);
}

TEST(Geometry, ContainsCoversTheCapsAsWellAsTheBody)
{
    Capsule const capsule{ { 0.0, 0.0 }, { 10.0, 0.0 }, 2.0 };

    EXPECT_TRUE(contains(capsule, { 5.0, 1.5 }));    // beside the body
    EXPECT_TRUE(contains(capsule, { -1.5, 0.0 }));   // inside the near cap
    EXPECT_TRUE(contains(capsule, { 11.5, 0.0 }));   // inside the far cap
    EXPECT_FALSE(contains(capsule, { 5.0, 2.5 }));   // clear of the body
    EXPECT_FALSE(contains(capsule, { -2.5, 0.0 }));  // clear of the cap
}

// A circle is the case where the segment has no length, so the round-obstacle
// path through all of this has to keep working.
TEST(Geometry, ADegenerateCapsuleBehavesLikeACircle)
{
    Capsule const buoy{ { 4.0, 0.0 }, { 4.0, 0.0 }, 1.0 };

    EXPECT_NEAR(length(buoy), 0.0, 1e-12);
    EXPECT_TRUE(hits(buoy, { 0.0, 0.5 }, { 8.0, 0.5 }));
    EXPECT_FALSE(hits(buoy, { 0.0, 1.5 }, { 8.0, 1.5 }));
}
