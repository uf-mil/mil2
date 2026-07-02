#include <gtest/gtest.h>

#include <cmath>

#include "search_pattern.hpp"

using namespace search_pattern;

TEST(SpiralWaypoints, EmptyOnBadArgs)
{
    EXPECT_TRUE(spiral_waypoints(0.0, 2.0).empty());
    EXPECT_TRUE(spiral_waypoints(0.5, 0.0).empty());
}

TEST(SpiralWaypoints, FirstStepsFollowSquareSpiral)
{
    auto p = spiral_waypoints(1.0, 10.0);
    ASSERT_GE(p.size(), 4u);
    EXPECT_NEAR(p[0].first, 1.0, 1e-9);  // +x
    EXPECT_NEAR(p[0].second, 0.0, 1e-9);
    EXPECT_NEAR(p[1].first, 1.0, 1e-9);  // +y
    EXPECT_NEAR(p[1].second, 1.0, 1e-9);
    EXPECT_NEAR(p[2].first, 0.0, 1e-9);  // -x
    EXPECT_NEAR(p[2].second, 1.0, 1e-9);
}

TEST(SpiralWaypoints, AllWithinRadius)
{
    double R = 2.0;
    auto p = spiral_waypoints(0.3, R);
    ASSERT_FALSE(p.empty());
    for (auto const& [x, y] : p)
        EXPECT_LE(std::hypot(x, y), R + 1e-9);
}

TEST(SpiralWaypoints, TighterStepGivesMoreWaypoints)
{
    EXPECT_GT(spiral_waypoints(0.2, 2.0).size(), spiral_waypoints(0.8, 2.0).size());
}
