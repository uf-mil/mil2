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

/// Below this an inflated capsule is a circle and needs only one ring of
/// corners rather than one around each end.
constexpr double kDegenerate = 1e-6;

bool route_is_clear(Point a, Point b, std::vector<Capsule> const& obstacles)
{
    return std::none_of(obstacles.begin(), obstacles.end(),
                        [&](Capsule const& capsule) { return hits(capsule, a, b); });
}

/// How far out from the axis to place the corners around an end cap.
///
/// They go on the polygon that circumscribes the cap rather than on the cap
/// itself. Corners placed on it are the vertices of an inscribed polygon, and
/// every chord of an inscribed polygon cuts inside - so no two adjacent
/// corners could see each other, and there would be no way around an obstacle
/// at all. Pushing them out by 1/cos(pi/n) makes those chords tangent instead,
/// and the last thousandth clears the tangency.
double corner_ring_radius(double radius, int corners)
{
    return radius * 1.001 / std::cos(M_PI / static_cast<double>(corners));
}

void add_ring(std::vector<Point>& nodes, Point centre, double ring, int corners)
{
    double const step = 2.0 * M_PI / static_cast<double>(corners);
    for (int i = 0; i < corners; ++i)
    {
        double const angle = step * static_cast<double>(i);
        nodes.push_back({ centre.x + ring * std::cos(angle), centre.y + ring * std::sin(angle) });
    }
}

/// Start, goal, then a ring of candidate corners around each end cap.
///
/// Corners only at the caps is enough. Going around a capsule means rounding
/// one cap, running parallel to the flat side, and rounding the other - and a
/// segment between two corners at the same angle on opposite caps sits a whole
/// ring radius off the axis, so the flat side needs no corners of its own.
std::vector<Point> build_nodes(Point start, Point goal, std::vector<Capsule> const& obstacles, int corners_per_obstacle)
{
    std::vector<Point> nodes;
    nodes.reserve(2 + obstacles.size() * 2 * static_cast<std::size_t>(corners_per_obstacle));
    nodes.push_back(start);
    nodes.push_back(goal);

    for (Capsule const& capsule : obstacles)
    {
        double const ring = corner_ring_radius(capsule.radius, corners_per_obstacle);
        add_ring(nodes, capsule.a, ring, corners_per_obstacle);
        if (length(capsule) >= kDegenerate)
        {
            add_ring(nodes, capsule.b, ring, corners_per_obstacle);
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
std::vector<Point> string_pull(Point start, std::vector<Point> const& route, std::vector<Capsule> const& obstacles)
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
            if (route_is_clear(from, route[j], obstacles))
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
    std::vector<Capsule> grown;
    grown.reserve(obstacles.size());
    for (Obstacle const& obstacle : obstacles)
    {
        Capsule const capsule = inflate(obstacle.shape, config_.inflation);
        if (!contains(capsule, start) && !contains(capsule, goal))
        {
            grown.push_back(capsule);
        }
    }

    // Straight there, when nothing is in the way. The common case on open
    // water, and it costs one test per obstacle.
    if (route_is_clear(start, goal, grown))
    {
        return { goal };
    }

    std::vector<Point> const nodes = build_nodes(start, goal, grown, config_.corners_per_obstacle);
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
            if (candidate >= cost[next] || !route_is_clear(nodes[current], nodes[next], grown))
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

    return string_pull(start, route, grown);
}

}  // namespace prop_planner
