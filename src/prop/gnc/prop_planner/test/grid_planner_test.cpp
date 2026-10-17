#include "prop_planner/grid_planner.hpp"

#include <gtest/gtest.h>

#include "prop_planner/evidence_grid.hpp"

using namespace prop_planner;

namespace
{

Grid open_water()
{
    Grid g;
    g.width = g.height = 80;
    g.resolution = 0.25;
    g.origin = { -10.0, -10.0 };
    g.cells.assign(80 * 80, 0);
    return g;
}

void wall(Grid& g, double x, double y0, double y1)
{
    for (double y = y0; y <= y1; y += g.resolution)
    {
        g.cells[g.index(g.cell_of({ x, y }))] = 100;
    }
}

}  // namespace

TEST(GridPlanner, OpenWaterIsAStraightLine)
{
    GridPlanner const planner({});
    Grid const g = open_water();
    Costmap const map = planner.costmap(g, { -5.0, 0.0 }, { 5.0, 0.0 });
    auto const route = planner.plan(map, { -5.0, 0.0 }, { 5.0, 0.0 });
    ASSERT_EQ(route.size(), 2u);
    EXPECT_NEAR(route_cost(map, route), 10.0, 1e-6);
}

TEST(GridPlanner, RoutesAroundAWallWithFewCorners)
{
    GridPlanner const planner({});
    Grid g = open_water();
    wall(g, 0.0, -4.0, 4.0);
    Costmap const map = planner.costmap(g, { -6.0, 0.0 }, { 6.0, 0.0 });
    auto const route = planner.plan(map, { -6.0, 0.0 }, { 6.0, 0.0 });
    ASSERT_GE(route.size(), 3u);
    EXPECT_LE(route.size(), 6u);
    EXPECT_LT(route_cost(map, route), Costmap::kLethal);
}

TEST(GridPlanner, StartInsideTheMarginStillGetsOut)
{
    GridPlanner const planner({});
    Grid g = open_water();
    wall(g, 0.0, -4.0, 4.0);
    Point const start{ -0.4, 0.0 };
    Costmap const map = planner.costmap(g, start, { 6.0, 0.0 });
    EXPECT_FALSE(planner.plan(map, start, { 6.0, 0.0 }).empty());
}

TEST(EvidenceGrid, HitsBuildUpAndMissesWearDown)
{
    EvidenceGrid grid({ 0.25, 20.0, 25, 10, 8.0 });
    Point const sensor{ 0.0, 0.0 };
    Point const buoy{ 3.0, 0.0 };
    grid.follow(sensor);
    for (int i = 0; i < 3; ++i)
    {
        grid.integrate({ buoy }, sensor);
    }
    Grid const& g = grid.grid();
    EXPECT_GE(g.cells[g.index(g.cell_of(buoy))], 60);

    for (int i = 0; i < 8; ++i)
    {
        grid.integrate({}, sensor);
    }
    EXPECT_EQ(g.cells[g.index(g.cell_of(buoy))], 0);
}

TEST(EvidenceGrid, WindowFollowsTheBoatAndKeepsWhatOverlaps)
{
    EvidenceGrid grid({ 0.25, 20.0, 25, 10, 8.0 });
    grid.follow({ 0.0, 0.0 });
    grid.integrate({ { 4.0, 0.0 } }, { 0.0, 0.0 });
    grid.follow({ 8.0, 0.0 });
    Grid const& g = grid.grid();
    EXPECT_NEAR(g.origin.x, -2.0, 1e-9);
    EXPECT_EQ(g.cells[g.index(g.cell_of({ 4.0, 0.0 }))], 25);
}

TEST(EvidenceGrid, RecoversFromAWildFirstPosition)
{
    EvidenceGrid grid({ 0.25, 20.0, 25, 10, 8.0 });
    grid.follow({ 1e24, -1e28 });
    grid.follow({ 0.0, 0.0 });
    grid.integrate({ { 4.0, 0.0 } }, { 0.0, 0.0 });
    Grid const& g = grid.grid();
    EXPECT_NEAR(g.origin.x, -10.0, 1e-9);
    EXPECT_EQ(g.cells[g.index(g.cell_of({ 4.0, 0.0 }))], 25);
}

TEST(GridPlanner, KeepsClearanceFromASingleBuoy)
{
    GridPlanner const planner({});
    Grid g = open_water();
    for (Point const p : { Point{ 2.0, 5.0 }, Point{ 2.2, 5.0 }, Point{ 2.0, 5.2 }, Point{ 2.2, 5.2 } })
    {
        g.cells[g.index(g.cell_of(p))] = 100;
    }
    Point const start{ 0.0, 0.0 };
    Point const goal{ 4.0, 10.0 };
    Costmap const map = planner.costmap(g, start, goal);
    auto const route = planner.plan(map, start, goal);
    ASSERT_GE(route.size(), 2u);

    double closest = 1e9;
    for (std::size_t i = 1; i < route.size(); ++i)
    {
        closest = std::min(closest, distance_to_segment({ 2.1, 5.1 }, route[i - 1], route[i]));
    }
    EXPECT_GE(closest, 1.5) << "route has " << route.size() << " points";
}
