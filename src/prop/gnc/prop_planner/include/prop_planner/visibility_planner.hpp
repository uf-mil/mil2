#pragma once

#include <cmath>
#include <vector>

#include "prop_planner/obstacle_map.hpp"

namespace prop_planner
{

struct Point
{
    double x{ 0.0 };
    double y{ 0.0 };
};

inline double distance(Point a, Point b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

// The shortest route past a handful of round obstacles.
//
// A course holds tens of buoys, not thousands of cells, and on that shape a
// visibility graph beats a grid on every axis that matters: it is exact rather
// than quantised, it costs microseconds to search, and what it returns is
// already a short list of corners. That last point is the one that decides it -
// guidance drives straight legs between waypoints and steers by a lookahead, so
// a handful of corners is exactly its input, while a grid planner would hand it
// a dense cell path to decimate and smooth first.
//
// Every obstacle is grown by the hull's half beam plus a safety margin, and the
// route is then the shortest polyline from start to goal that enters no grown
// circle.
class VisibilityPlanner
{
  public:
    struct Config
    {
        /// Added to every obstacle radius: half the hull's beam, plus however
        /// close you are willing to pass.
        double inflation{ 2.0 };
        /// Corners sampled around each obstacle. More of them means routes that
        /// hug obstacles more closely, at a quadratic cost in the search.
        int corners_per_obstacle{ 8 };
    };

    explicit VisibilityPlanner(Config config) : config_(config)
    {
    }

    /// Waypoints from just after start through to goal, or an empty vector when
    /// there is no route. Obstacles already overlapping start or goal are
    /// ignored: the boat is where it is, and reporting failure would not move it.
    std::vector<Point> plan(Point start, Point goal, std::vector<Obstacle> const& obstacles) const;

  private:
    Config config_;
};

}  // namespace prop_planner
