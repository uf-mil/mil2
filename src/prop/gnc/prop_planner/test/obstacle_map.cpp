#include "prop_planner/obstacle_map.hpp"

#include <gtest/gtest.h>

using prop_planner::Obstacle;
using prop_planner::ObstacleMap;

namespace
{

// Defaults: merge 2 m, gain 0.2, min_hits 3, max_radius 5 m, capacity 256.
ObstacleMap::Config defaults()
{
    return ObstacleMap::Config{};
}

}  // namespace

// The point of the whole class: many looks at one buoy stay one buoy.
TEST(ObstacleMap, RepeatedViewsOfOneBuoyStayOneEntry)
{
    ObstacleMap map(defaults());
    for (int i = 0; i < 10; ++i)
    {
        // Jitter well inside merge_distance, as a walking centroid would.
        map.observe(10.0 + 0.1 * (i % 3), 5.0, 0.5);
    }

    ASSERT_EQ(map.all().size(), 1u);
    EXPECT_EQ(map.all()[0].hits, 10);
    EXPECT_NEAR(map.all()[0].x, 10.1, 0.2);
}

TEST(ObstacleMap, BuoysFartherApartThanMergeDistanceStaySeparate)
{
    ObstacleMap map(defaults());
    map.observe(10.0, 5.0, 0.5);
    map.observe(30.0, 5.0, 0.5);

    EXPECT_EQ(map.all().size(), 2u);
}

// The gap either side of merge_distance, which is the parameter most likely to
// be adjusted. Just inside merges, just outside does not.
TEST(ObstacleMap, MergeDistanceIsTheBoundary)
{
    ObstacleMap map(defaults());
    map.observe(0.0, 0.0, 0.5);
    map.observe(1.9, 0.0, 0.5);
    ASSERT_EQ(map.all().size(), 1u);

    map.observe(50.0, 0.0, 0.5);
    map.observe(52.1, 0.0, 0.5);
    EXPECT_EQ(map.all().size(), 3u);
}

// min_hits is what keeps wave crests out of the map.
TEST(ObstacleMap, EntriesAreOnlyConfirmedAfterMinHits)
{
    ObstacleMap map(defaults());

    map.observe(10.0, 5.0, 0.5);
    EXPECT_TRUE(map.confirmed().empty());

    map.observe(10.0, 5.0, 0.5);
    EXPECT_TRUE(map.confirmed().empty());

    map.observe(10.0, 5.0, 0.5);
    EXPECT_EQ(map.confirmed().size(), 1u);
}

TEST(ObstacleMap, ConfirmedIsASubsetOfAll)
{
    ObstacleMap map(defaults());
    for (int i = 0; i < 5; ++i)
    {
        map.observe(0.0, 0.0, 0.5);
    }
    map.observe(40.0, 0.0, 0.5);  // seen once, still pending

    EXPECT_EQ(map.all().size(), 2u);
    EXPECT_EQ(map.confirmed().size(), 1u);
}

// One frame that merges two buoys into a blob must not wall off the course.
TEST(ObstacleMap, RadiusIsClampedToMaxRadius)
{
    ObstacleMap map(defaults());
    map.observe(10.0, 5.0, 0.5);
    map.observe(10.0, 5.0, 40.0);

    EXPECT_LT(map.all()[0].radius, 5.1);
}

// A position is blended in rather than replacing, so one bad centroid moves
// the entry by only its share.
TEST(ObstacleMap, PositionIsBlendedNotReplaced)
{
    ObstacleMap map(defaults());
    map.observe(0.0, 0.0, 0.5);
    map.observe(1.0, 0.0, 0.5);

    // gain 0.2, so 0.0 + 0.2 * (1.0 - 0.0).
    EXPECT_NEAR(map.all()[0].x, 0.2, 1e-9);
}

TEST(ObstacleMap, CapacityEvictsTheLeastSeenEntry)
{
    ObstacleMap::Config config = defaults();
    config.capacity = 3;
    ObstacleMap map(config);

    // Three well-separated entries; the first is seen far more often.
    for (int i = 0; i < 9; ++i)
    {
        map.observe(0.0, 0.0, 0.5);
    }
    map.observe(20.0, 0.0, 0.5);
    map.observe(40.0, 0.0, 0.5);
    ASSERT_EQ(map.all().size(), 3u);

    map.observe(60.0, 0.0, 0.5);
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
        map.observe(10.0, 5.0, 0.5);
    }
    ASSERT_EQ(map.confirmed().size(), 1u);

    // Many frames in which this buoy is not seen at all.
    for (int i = 0; i < 100; ++i)
    {
        map.observe(80.0, 80.0, 0.5);
    }

    EXPECT_EQ(map.confirmed().size(), 2u);
}
