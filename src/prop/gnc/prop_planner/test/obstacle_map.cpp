#include "prop_planner/obstacle_map.hpp"

#include <gtest/gtest.h>

#include <cmath>

using prop_planner::Capsule;
using prop_planner::length;
using prop_planner::merge_capsules;
using prop_planner::Obstacle;
using prop_planner::ObstacleMap;
using prop_planner::Point;

namespace
{

// Defaults: merge 2 m, gain 0.2, min_hits 3, max_radius 5 m, max_length 20 m,
// deadband 0.3 m, capacity 256.
ObstacleMap::Config defaults()
{
    return ObstacleMap::Config{};
}

/// A round detection: the degenerate capsule a buoy produces.
Capsule round_at(double x, double y, double radius)
{
    return Capsule{ { x, y }, { x, y }, radius };
}

/// An elongated detection along the x axis.
Capsule bar(double x1, double x2, double y, double radius)
{
    return Capsule{ { x1, y }, { x2, y }, radius };
}

}  // namespace

// The point of the whole class: many looks at one buoy stay one buoy.
TEST(ObstacleMap, RepeatedViewsOfOneBuoyStayOneEntry)
{
    ObstacleMap map(defaults());
    for (int i = 0; i < 10; ++i)
    {
        // Jitter well inside merge_distance, as a walking centroid would.
        map.observe(round_at(10.0 + 0.1 * (i % 3), 5.0, 0.5));
    }

    ASSERT_EQ(map.all().size(), 1u);
    EXPECT_EQ(map.all()[0].hits, 10);
    EXPECT_NEAR(map.all()[0].shape.a.x, 10.1, 0.2);
}

TEST(ObstacleMap, BuoysFartherApartThanMergeDistanceStaySeparate)
{
    ObstacleMap map(defaults());
    map.observe(round_at(10.0, 5.0, 0.5));
    map.observe(round_at(30.0, 5.0, 0.5));

    EXPECT_EQ(map.all().size(), 2u);
}

// The gap either side of merge_distance, which is the parameter most likely to
// be adjusted. Just inside merges, just outside does not.
TEST(ObstacleMap, MergeDistanceIsTheBoundary)
{
    ObstacleMap map(defaults());
    map.observe(round_at(0.0, 0.0, 0.5));
    map.observe(round_at(1.9, 0.0, 0.5));
    ASSERT_EQ(map.all().size(), 1u);

    map.observe(round_at(50.0, 0.0, 0.5));
    map.observe(round_at(52.1, 0.0, 0.5));
    EXPECT_EQ(map.all().size(), 3u);
}

// min_hits is what keeps wave crests out of the map.
TEST(ObstacleMap, EntriesAreOnlyConfirmedAfterMinHits)
{
    ObstacleMap map(defaults());

    map.observe(round_at(10.0, 5.0, 0.5));
    EXPECT_TRUE(map.confirmed().empty());

    map.observe(round_at(10.0, 5.0, 0.5));
    EXPECT_TRUE(map.confirmed().empty());

    map.observe(round_at(10.0, 5.0, 0.5));
    EXPECT_EQ(map.confirmed().size(), 1u);
}

TEST(ObstacleMap, ConfirmedIsASubsetOfAll)
{
    ObstacleMap map(defaults());
    for (int i = 0; i < 5; ++i)
    {
        map.observe(round_at(0.0, 0.0, 0.5));
    }
    map.observe(round_at(40.0, 0.0, 0.5));  // seen once, still pending

    EXPECT_EQ(map.all().size(), 2u);
    EXPECT_EQ(map.confirmed().size(), 1u);
}

// One frame that merges two buoys into a blob must not wall off the course.
TEST(ObstacleMap, RadiusIsClampedToMaxRadius)
{
    ObstacleMap map(defaults());
    map.observe(round_at(10.0, 5.0, 0.5));
    map.observe(round_at(10.0, 5.0, 40.0));

    EXPECT_LT(map.all()[0].shape.radius, 5.1);
}

// A position is blended in rather than replacing, so one bad centroid moves
// the entry by only its share.
TEST(ObstacleMap, PositionIsBlendedNotReplaced)
{
    ObstacleMap map(defaults());
    map.observe(round_at(0.0, 0.0, 0.5));
    map.observe(round_at(1.0, 0.0, 0.5));

    // gain 0.2, so 0.0 + 0.2 * (1.0 - 0.0).
    EXPECT_NEAR(map.all()[0].shape.a.x, 0.2, 1e-9);
}

TEST(ObstacleMap, CapacityEvictsTheLeastSeenEntry)
{
    ObstacleMap::Config config = defaults();
    config.capacity = 3;
    ObstacleMap map(config);

    // Three well-separated entries; the first is seen far more often.
    for (int i = 0; i < 9; ++i)
    {
        map.observe(round_at(0.0, 0.0, 0.5));
    }
    map.observe(round_at(20.0, 0.0, 0.5));
    map.observe(round_at(40.0, 0.0, 0.5));
    ASSERT_EQ(map.all().size(), 3u);

    map.observe(round_at(60.0, 0.0, 0.5));
    ASSERT_EQ(map.all().size(), 3u);

    // The busiest entry survives; one of the singletons was dropped.
    bool kept_busiest = false;
    for (Obstacle const& o : map.all())
    {
        if (o.hits >= 9)
        {
            kept_busiest = true;
        }
    }
    EXPECT_TRUE(kept_busiest);
}

// The class exists because pcl_tracker forgets. Absence must not erase.
TEST(ObstacleMap, NothingIsForgottenWhenObservationsStop)
{
    ObstacleMap map(defaults());
    for (int i = 0; i < 5; ++i)
    {
        map.observe(round_at(10.0, 5.0, 0.5));
    }
    ASSERT_EQ(map.confirmed().size(), 1u);

    // Many frames in which this buoy is not seen at all.
    for (int i = 0; i < 100; ++i)
    {
        map.observe(round_at(80.0, 80.0, 0.5));
    }

    EXPECT_EQ(map.confirmed().size(), 2u);
}

// A segment can arrive either way round. Pairing the ends naively drags both
// toward the middle and the capsule vanishes.
TEST(MergeCapsules, AFlippedObservationDoesNotCollapseTheCapsule)
{
    Capsule const stored{ { 0.0, 0.0 }, { 10.0, 0.0 }, 1.0 };
    Capsule const flipped{ { 10.0, 0.0 }, { 0.0, 0.0 }, 1.0 };

    Capsule const merged = merge_capsules(stored, flipped, 0.2, 0.3);

    EXPECT_NEAR(length(merged), 10.0, 1e-6);
}

TEST(MergeCapsules, RadiusAndCentreAreBlendedTowardTheObservation)
{
    Capsule const stored{ { 0.0, 0.0 }, { 0.0, 0.0 }, 1.0 };
    Capsule const observed{ { 1.0, 0.0 }, { 1.0, 0.0 }, 2.0 };

    Capsule const merged = merge_capsules(stored, observed, 0.2, 0.3);

    EXPECT_NEAR(merged.a.x, 0.2, 1e-9);
    EXPECT_NEAR(merged.radius, 1.2, 1e-9);
}

// Each view of a long object is a slice of it, so the stored extent has to
// reach past what any single frame showed.
TEST(ObstacleMap, ExtentAccumulatesAcrossPartialViews)
{
    ObstacleMap map(defaults());

    for (int i = 0; i < 60; ++i)
    {
        map.observe(bar(-3.0, 3.0, 0.0, 0.5));
    }
    ASSERT_EQ(map.all().size(), 1u);
    double const first_view = length(map.all()[0].shape);
    EXPECT_NEAR(first_view, 6.0, 0.5);

    // A second stretch of the same dock, overlapping the first.
    for (int i = 0; i < 60; ++i)
    {
        map.observe(bar(3.0, 9.0, 0.0, 0.5));
    }

    ASSERT_EQ(map.all().size(), 1u) << "the two views are the same object";
    EXPECT_GT(length(map.all()[0].shape), first_view + 4.0) << "the second view should have extended it";
    EXPECT_NEAR(length(map.all()[0].shape), 12.0, 1.0);
}

// Centres 8 m apart, far beyond merge_distance, but the axes lie on top of
// each other. Associating on centres alone would file these as two obstacles.
TEST(ObstacleMap, AssociationIsByAxisNotByCentre)
{
    ObstacleMap map(defaults());

    map.observe(bar(-9.0, 1.0, 0.0, 0.5));
    map.observe(bar(-1.0, 9.0, 0.0, 0.5));

    EXPECT_EQ(map.all().size(), 1u);
}

// Without the deadband, observation noise would slowly stretch a round buoy
// into a capsule, because extent only ever grows.
TEST(ObstacleMap, JitterDoesNotSmearARoundBuoyIntoACapsule)
{
    ObstacleMap map(defaults());

    for (int i = 0; i < 200; ++i)
    {
        double const jitter = 0.1 * std::sin(0.7 * i);
        map.observe(round_at(10.0 + jitter, 5.0 - jitter, 0.5));
    }

    ASSERT_EQ(map.all().size(), 1u);
    EXPECT_LT(length(map.all()[0].shape), 0.2) << "should still be round";
}

// The valve a convex hull cannot have: a polygon has no single number to clamp.
TEST(ObstacleMap, LengthIsClampedToMaxLength)
{
    ObstacleMap map(defaults());

    for (int i = 0; i < 40; ++i)
    {
        map.observe(bar(-30.0, 30.0, 0.0, 0.5));
    }

    ASSERT_EQ(map.all().size(), 1u);
    EXPECT_LE(length(map.all()[0].shape), defaults().max_length + 1e-6);
}

TEST(ObstacleMap, RoundObservationsStayRound)
{
    ObstacleMap map(defaults());

    for (int i = 0; i < 30; ++i)
    {
        map.observe(round_at(10.0, 5.0, 0.5));
    }

    ASSERT_EQ(map.all().size(), 1u);
    EXPECT_NEAR(length(map.all()[0].shape), 0.0, 1e-6);
    EXPECT_NEAR(map.all()[0].shape.radius, 0.5, 1e-6);
}
