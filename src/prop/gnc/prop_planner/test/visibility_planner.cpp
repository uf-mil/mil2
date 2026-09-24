#include "prop_planner/visibility_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

using prop_planner::Obstacle;
using prop_planner::Point;
using prop_planner::VisibilityPlanner;

namespace
{

// Defaults: inflation 2 m, 8 corners per obstacle.
VisibilityPlanner::Config defaults()
{
    return VisibilityPlanner::Config{};
}

Obstacle buoy(double x, double y, double radius = 0.5)
{
    return Obstacle{ x, y, radius, 9 };
}

/// Closest approach of the segment ab to p, written out here rather than
/// reused from the planner: a route checked with the planner's own helper
/// would agree with it even where both are wrong.
double distance_to_segment(Point p, Point a, Point b)
{
    double const dx = b.x - a.x;
    double const dy = b.y - a.y;
    double const length_squared = dx * dx + dy * dy;
    if (length_squared < 1e-12)
    {
        return std::hypot(p.x - a.x, p.y - a.y);
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared;
    t = std::max(0.0, std::min(1.0, t));
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

/// Smallest clearance the route leaves past every obstacle's margin, ignoring
/// the ones already overlapping the start, which the planner drops on purpose.
double worst_clearance(Point start, std::vector<Point> const& route, std::vector<Obstacle> const& obstacles,
                       double inflation)
{
    double worst = std::numeric_limits<double>::infinity();
    Point from = start;
    for (Point const& waypoint : route)
    {
        for (Obstacle const& obstacle : obstacles)
        {
            double const margin = obstacle.radius + inflation;
            if (std::hypot(obstacle.x - start.x, obstacle.y - start.y) < margin)
            {
                continue;
            }
            double const gap = distance_to_segment({ obstacle.x, obstacle.y }, from, waypoint) - margin;
            worst = std::min(worst, gap);
        }
        from = waypoint;
    }
    return worst;
}

}  // namespace

TEST(VisibilityPlanner, OpenWaterIsASingleWaypoint)
{
    VisibilityPlanner planner(defaults());
    std::vector<Point> const route = planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, {});

    ASSERT_EQ(route.size(), 1u);
    EXPECT_NEAR(route[0].x, 50.0, 1e-9);
    EXPECT_NEAR(route[0].y, 0.0, 1e-9);
}

TEST(VisibilityPlanner, RoutesAroundABuoyOnTheDirectLine)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ buoy(25.0, 0.0) };
    std::vector<Point> const route = planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);

    ASSERT_FALSE(route.empty());
    EXPECT_GT(route.size(), 1u) << "a straight line would pass through the buoy";
    EXPECT_GE(worst_clearance({ 0.0, 0.0 }, route, obstacles, defaults().inflation), -1e-9);
}

TEST(VisibilityPlanner, DrivesThroughAGateThatIsWideEnough)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ buoy(25.0, 6.0, 0.4), buoy(25.0, -6.0, 0.4) };
    std::vector<Point> const route = planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);

    ASSERT_EQ(route.size(), 1u) << "12 m apart clears a 2.4 m margin either side";
    EXPECT_GE(worst_clearance({ 0.0, 0.0 }, route, obstacles, defaults().inflation), -1e-9);
}

TEST(VisibilityPlanner, GoesAroundAGateThatIsTooNarrow)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ buoy(25.0, 1.5, 0.4), buoy(25.0, -1.5, 0.4) };
    std::vector<Point> const route = planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);

    ASSERT_FALSE(route.empty());
    EXPECT_GT(route.size(), 1u);
    EXPECT_GE(worst_clearance({ 0.0, 0.0 }, route, obstacles, defaults().inflation), -1e-9);
}

// Reporting no route is correct; inventing one through a buoy is not.
TEST(VisibilityPlanner, ReportsNoRouteWhenTheGoalIsWalledIn)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> obstacles;
    for (int i = 0; i < 16; ++i)
    {
        double const angle = i * 2.0 * M_PI / 16.0;
        obstacles.push_back(buoy(50.0 + 6.0 * std::cos(angle), 6.0 * std::sin(angle), 0.4));
    }

    EXPECT_TRUE(planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles).empty());
}

// The boat is where it is; refusing to plan would not move it.
TEST(VisibilityPlanner, StillPlansWhenABuoyIsAlreadyInsideTheMargin)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ buoy(1.0, 0.5), buoy(25.0, 0.0) };
    std::vector<Point> const route = planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);

    ASSERT_FALSE(route.empty());
    EXPECT_GE(worst_clearance({ 0.0, 0.0 }, route, obstacles, defaults().inflation), -1e-9);
}

// Corners sit on the circumscribed polygon so adjacent ones can see each
// other. On the inscribed one every chord cuts inside and no route exists.
TEST(VisibilityPlanner, CornerRingLeavesAdjacentCornersMutuallyVisible)
{
    for (int corners : { 6, 8, 12, 16 })
    {
        VisibilityPlanner::Config config = defaults();
        config.corners_per_obstacle = corners;
        VisibilityPlanner planner(config);

        std::vector<Obstacle> const obstacles{ buoy(25.0, 0.0, 3.0) };
        std::vector<Point> const route = planner.plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);

        ASSERT_FALSE(route.empty()) << "no way around the obstacle with " << corners << " corners";
        EXPECT_GE(worst_clearance({ 0.0, 0.0 }, route, obstacles, config.inflation), -1e-9)
            << "clipped the obstacle with " << corners << " corners";
    }
}

// Guidance restarts its leg on every plan, so the same scene twice has to give
// the same route or the boat would be reset by a flickering path.
TEST(VisibilityPlanner, IsDeterministic)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ buoy(25.0, 1.5, 0.4), buoy(25.0, -1.5, 0.4), buoy(40.0, 3.0) };

    std::vector<Point> const first = planner.plan({ 0.0, 0.0 }, { 60.0, 0.0 }, obstacles);
    std::vector<Point> const second = planner.plan({ 0.0, 0.0 }, { 60.0, 0.0 }, obstacles);

    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        EXPECT_NEAR(first[i].x, second[i].x, 1e-12);
        EXPECT_NEAR(first[i].y, second[i].y, 1e-12);
    }
}

TEST(VisibilityPlanner, RaisingInflationWidensTheDetour)
{
    std::vector<Obstacle> const obstacles{ buoy(25.0, 0.0) };

    VisibilityPlanner::Config narrow = defaults();
    narrow.inflation = 2.0;
    VisibilityPlanner::Config wide = defaults();
    wide.inflation = 6.0;

    std::vector<Point> const tight = VisibilityPlanner(narrow).plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);
    std::vector<Point> const loose = VisibilityPlanner(wide).plan({ 0.0, 0.0 }, { 50.0, 0.0 }, obstacles);

    ASSERT_FALSE(tight.empty());
    ASSERT_FALSE(loose.empty());
    EXPECT_GT(std::abs(loose[0].y), std::abs(tight[0].y));
}

// A full course has to stay well inside the 1 Hz replan budget.
TEST(VisibilityPlanner, SearchesAFullCourseQuickly)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> obstacles;
    for (int i = 0; i < 30; ++i)
    {
        obstacles.push_back(buoy(-40.0 + 2.7 * i, 18.0 * std::sin(0.7 * i)));
    }

    auto const started = std::chrono::steady_clock::now();
    for (int i = 0; i < 50; ++i)
    {
        planner.plan({ -60.0, -50.0 }, { 60.0, 50.0 }, obstacles);
    }
    auto const elapsed = std::chrono::steady_clock::now() - started;

    double const per_plan_ms = std::chrono::duration<double, std::milli>(elapsed).count() / 50.0;
    EXPECT_LT(per_plan_ms, 100.0) << "took " << per_plan_ms << " ms per plan";
}
