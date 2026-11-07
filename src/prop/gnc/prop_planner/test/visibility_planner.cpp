#include "prop_planner/visibility_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

using prop_planner::Capsule;
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
    return Obstacle{ Capsule{ { x, y }, { x, y }, radius }, 9 };
}

/// An elongated obstacle: a dock or a wall rather than a buoy.
Obstacle bar(double x1, double y1, double x2, double y2, double radius = 1.0)
{
    return Obstacle{ Capsule{ { x1, y1 }, { x2, y2 }, radius }, 9 };
}

/// Closest approach of the segment ab to p, written out here rather than
/// reused from the package's own helper: a route checked with the code that
/// produced it would agree with it even where both are wrong.
double ref_distance_to_segment(Point p, Point a, Point b)
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

double cross(Point a, Point b, Point c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/// Closest approach between two segments, zero where they properly cross.
double ref_segment_distance(Point a, Point b, Point c, Point d)
{
    bool const crossing =
        ((cross(a, b, c) > 0.0) != (cross(a, b, d) > 0.0)) && ((cross(c, d, a) > 0.0) != (cross(c, d, b) > 0.0));
    if (crossing)
    {
        return 0.0;
    }
    return std::min({ ref_distance_to_segment(a, c, d), ref_distance_to_segment(b, c, d),
                      ref_distance_to_segment(c, a, b), ref_distance_to_segment(d, a, b) });
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
            double const margin = obstacle.shape.radius + inflation;
            if (ref_distance_to_segment(start, obstacle.shape.a, obstacle.shape.b) < margin)
            {
                continue;  // already inside the margin; the planner drops these
            }
            double const gap = ref_segment_distance(from, waypoint, obstacle.shape.a, obstacle.shape.b) - margin;
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

// The reason the shape changed. A circle enclosing this dock would have a
// radius of just over 10 m and would block a corridor that is actually open.
TEST(VisibilityPlanner, PassesAlongsideALongObstacleThatACircleWouldBlock)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ bar(-9.0, 0.0, 9.0, 0.0) };

    std::vector<Point> const route = planner.plan({ -20.0, 6.0 }, { 20.0, 6.0 }, obstacles);

    ASSERT_EQ(route.size(), 1u) << "abeam the dock and clear of it, so a straight run";
    EXPECT_GE(worst_clearance({ -20.0, 6.0 }, route, obstacles, defaults().inflation), -1e-9);
}

TEST(VisibilityPlanner, RoutesAroundTheEndOfALongObstacle)
{
    VisibilityPlanner planner(defaults());
    std::vector<Obstacle> const obstacles{ bar(0.0, -8.0, 0.0, 8.0) };

    // Straight across is blocked by the length of it, so it has to go round.
    std::vector<Point> const route = planner.plan({ -15.0, 0.0 }, { 15.0, 0.0 }, obstacles);

    ASSERT_FALSE(route.empty());
    EXPECT_GT(route.size(), 1u);
    EXPECT_GE(worst_clearance({ -15.0, 0.0 }, route, obstacles, defaults().inflation), -1e-9);
}

TEST(VisibilityPlanner, CornersAtBothCapsLeaveARouteAroundAnElongatedObstacle)
{
    for (int corners : { 6, 8, 12 })
    {
        VisibilityPlanner::Config config = defaults();
        config.corners_per_obstacle = corners;
        VisibilityPlanner planner(config);

        std::vector<Obstacle> const obstacles{ bar(0.0, -6.0, 0.0, 6.0, 2.0) };
        std::vector<Point> const route = planner.plan({ -15.0, 0.0 }, { 15.0, 0.0 }, obstacles);

        ASSERT_FALSE(route.empty()) << "no way around with " << corners << " corners";
        EXPECT_GE(worst_clearance({ -15.0, 0.0 }, route, obstacles, config.inflation), -1e-9)
            << "clipped it with " << corners << " corners";
    }
}
