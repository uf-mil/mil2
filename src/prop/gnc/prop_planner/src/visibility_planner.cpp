#include "prop_planner/visibility_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace prop_planner
{
namespace
{

/// An obstacle grown by the safety margin: what the planner actually avoids.
struct Disc
{
    Point center;
    double radius{ 0.0 };
};

/// Closest approach of the segment ab to the point p.
double distance_to_segment(Point p, Point a, Point b)
{
    double const dx = b.x - a.x;
    double const dy = b.y - a.y;
    double const length_squared = dx * dx + dy * dy;

    // A segment of no length is just its endpoint.
    if (length_squared < 1e-12)
    {
        return distance(p, a);
    }

    // Project p onto the infinite line through a and b, then clamp the
    // projection back onto the segment itself.
    double const t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

bool segment_is_clear(Point a, Point b, std::vector<Disc> const& discs)
{
    return std::none_of(discs.begin(), discs.end(),
                        [&](Disc const& disc) { return distance_to_segment(disc.center, a, b) < disc.radius; });
}

/// Where to put the corners around a grown obstacle.
///
/// They go on the polygon that circumscribes the circle rather than on the
/// circle itself. Corners placed on the circle are the vertices of an inscribed
/// polygon, and every chord of an inscribed polygon cuts inside - so no two
/// adjacent corners could see each other, and there would be no way around an
/// obstacle at all. Pushing them out by 1/cos(pi/n) makes those chords tangent
/// instead, and the last thousandth clears the tangency.
double corner_ring_radius(double radius, int corners)
{
    return radius * 1.001 / std::cos(M_PI / static_cast<double>(corners));
}

/// Start, goal, then a ring of candidate corners around every obstacle.
std::vector<Point> build_nodes(Point start, Point goal, std::vector<Disc> const& discs, int corners_per_obstacle)
{
    std::vector<Point> nodes;
    nodes.reserve(2 + discs.size() * static_cast<std::size_t>(corners_per_obstacle));
    nodes.push_back(start);
    nodes.push_back(goal);

    double const step = 2.0 * M_PI / static_cast<double>(corners_per_obstacle);
    for (auto const& disc : discs)
    {
        double const ring = corner_ring_radius(disc.radius, corners_per_obstacle);
        for (int i = 0; i < corners_per_obstacle; ++i)
        {
            double const angle = step * static_cast<double>(i);
            nodes.push_back({ disc.center.x + ring * std::cos(angle), disc.center.y + ring * std::sin(angle) });
        }
    }
    return nodes;
}

/// Collapse runs of corners that the geometry did not need.
///
/// The graph's shortest path can still round an obstacle in more corners than
/// the straight lines require, because it may only turn at sampled points.
/// Walking forward to the furthest corner still in sight removes those, which
/// leaves guidance fewer legs to chase.
std::vector<Point> string_pull(Point start, std::vector<Point> const& route, std::vector<Disc> const& discs)
{
    std::vector<Point> pulled;
    pulled.reserve(route.size());

    Point from = start;
    std::size_t i = 0;
    while (i < route.size())
    {
        // Search back from the end so the first hit is the furthest one.
        std::size_t furthest = i;
        for (std::size_t j = route.size(); j-- > i;)
        {
            if (segment_is_clear(from, route[j], discs))
            {
                furthest = j;
                break;
            }
        }
        pulled.push_back(route[furthest]);
        from = route[furthest];
        i = furthest + 1;
    }
    return pulled;
}

}  // namespace

std::vector<Point> VisibilityPlanner::plan(Point start, Point goal, std::vector<Obstacle> const& obstacles) const
{
    // Grow every obstacle by the safety margin, and drop the ones already
    // overlapping an endpoint. Keeping those could only ever report "no route"
    // for a situation the planner cannot fix - the boat is already inside the
    // margin, and refusing to plan would leave it there.
    std::vector<Disc> discs;
    discs.reserve(obstacles.size());
    for (auto const& obstacle : obstacles)
    {
        Disc const disc{ { obstacle.x, obstacle.y }, obstacle.radius + config_.inflation };
        if (distance(disc.center, start) >= disc.radius && distance(disc.center, goal) >= disc.radius)
        {
            discs.push_back(disc);
        }
    }

    // Straight there, when nothing is in the way. The common case on open
    // water, and it costs one segment test.
    if (segment_is_clear(start, goal, discs))
    {
        return { goal };
    }

    std::vector<Point> const nodes = build_nodes(start, goal, discs, config_.corners_per_obstacle);
    std::size_t const count = nodes.size();
    constexpr std::size_t kStart = 0;
    constexpr std::size_t kGoal = 1;
    constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

    // A* over the visibility graph. Neighbours are generated on demand rather
    // than stored: the edge set is quadratic in the node count and a search
    // that reaches the goal early never looks at most of it.
    std::vector<double> cost(count, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> came_from(count, kNoParent);
    std::vector<bool> expanded(count, false);

    using Candidate = std::pair<double, std::size_t>;  // estimated total cost, node
    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<>> open;

    cost[kStart] = 0.0;
    open.emplace(distance(start, goal), kStart);

    while (!open.empty())
    {
        std::size_t const current = open.top().second;
        open.pop();

        if (current == kGoal)
        {
            break;
        }
        if (expanded[current])
        {
            continue;  // already reached by a cheaper route
        }
        expanded[current] = true;

        for (std::size_t next = 0; next < count; ++next)
        {
            if (next == current || expanded[next])
            {
                continue;
            }

            double const candidate = cost[current] + distance(nodes[current], nodes[next]);

            // Arithmetic before geometry: most neighbours fail on cost alone,
            // and that test is far cheaper than sweeping every obstacle.
            if (candidate >= cost[next] || !segment_is_clear(nodes[current], nodes[next], discs))
            {
                continue;
            }

            cost[next] = candidate;
            came_from[next] = current;
            open.emplace(candidate + distance(nodes[next], goal), next);
        }
    }

    if (came_from[kGoal] == kNoParent)
    {
        return {};  // walled in
    }

    std::vector<Point> route;
    for (std::size_t node = kGoal; node != kStart; node = came_from[node])
    {
        route.push_back(nodes[node]);
    }
    std::reverse(route.begin(), route.end());

    return string_pull(start, route, discs);
}

}  // namespace prop_planner
