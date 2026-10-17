#include "prop_planner/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace prop_planner
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();

double leg_cost(Costmap const& map, Point a, Point b)
{
    Grid const& g = *map.grid;
    double const len = distance(a, b);
    int const steps = std::max(1, static_cast<int>(std::ceil(len / (0.5 * g.resolution))));
    double total = 0.0;
    for (int k = 1; k <= steps; ++k)
    {
        double const t = static_cast<double>(k) / steps;
        Cell const c = g.cell_of({ a.x + t * (b.x - a.x), a.y + t * (b.y - a.y) });
        total += (len / steps) * (1.0 + (g.contains(c) ? map.cost[g.index(c)] : 0.0f));
    }
    return total;
}

bool passable(Costmap const& map, Cell c)
{
    return map.grid->contains(c) && map.cost[map.grid->index(c)] < Costmap::kLethal;
}

/// The passable cell nearest to c, searched in growing squares; c itself when
/// none is within reach.
Cell nearest_passable(Costmap const& map, Cell c, int reach)
{
    for (int r = 0; r <= reach; ++r)
    {
        for (int dy = -r; dy <= r; ++dy)
        {
            for (int dx = -r; dx <= r; ++dx)
            {
                Cell const n{ c.x + dx, c.y + dy };
                if (std::max(std::abs(dx), std::abs(dy)) == r && passable(map, n))
                {
                    return n;
                }
            }
        }
    }
    return c;
}

/// Drop every corner whose neighbours can be joined directly for no more than
/// the cells in between cost.
std::vector<Point> straighten(Costmap const& map, std::vector<Point> const& path)
{
    std::vector<double> cumulative(path.size(), 0.0);
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        cumulative[i] = cumulative[i - 1] + leg_cost(map, path[i - 1], path[i]);
    }

    std::vector<Point> out{ path.front() };
    for (std::size_t i = 0; i + 1 < path.size();)
    {
        std::size_t j = path.size() - 1;
        while (j > i + 1 && leg_cost(map, path[i], path[j]) > 1.02 * (cumulative[j] - cumulative[i]))
        {
            --j;
        }
        out.push_back(path[j]);
        i = j;
    }

    // A corner within a cell of the next point is quantisation, not a turn.
    for (std::size_t k = 1; k + 1 < out.size();)
    {
        bool const tiny = distance(out[k], out[k + 1]) < map.grid->resolution;
        if (tiny && leg_cost(map, out[k - 1], out[k + 1]) < Costmap::kLethal)
        {
            out.erase(out.begin() + static_cast<std::ptrdiff_t>(k));
        }
        else
        {
            ++k;
        }
    }
    return out;
}

}  // namespace

double route_cost(Costmap const& map, std::vector<Point> const& route)
{
    double total = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i)
    {
        total += leg_cost(map, route[i - 1], route[i]);
    }
    return total;
}

Costmap GridPlanner::costmap(Grid const& g, Point start, Point goal) const
{
    Costmap map{ &g, std::vector<float>(g.cells.size(), 0.0f) };

    // Distance to the nearest solid cell, out to clearance, by stamping a disc
    // around each one.
    int const reach = static_cast<int>(std::ceil(config_.clearance / g.resolution));
    std::vector<std::pair<Cell, float>> disc;
    for (int dy = -reach; dy <= reach; ++dy)
    {
        for (int dx = -reach; dx <= reach; ++dx)
        {
            float const d = static_cast<float>(std::hypot(dx, dy) * g.resolution);
            if (d <= config_.clearance)
            {
                disc.push_back({ { dx, dy }, d });
            }
        }
    }

    std::vector<float> nearest(g.cells.size(), std::numeric_limits<float>::infinity());
    for (int y = 0; y < g.height; ++y)
    {
        for (int x = 0; x < g.width; ++x)
        {
            if (g.cells[g.index({ x, y })] < config_.occupied)
            {
                continue;
            }
            for (auto const& [offset, d] : disc)
            {
                Cell const n{ x + offset.x, y + offset.y };
                if (g.contains(n))
                {
                    nearest[g.index(n)] = std::min(nearest[g.index(n)], d);
                }
            }
        }
    }

    double const band = std::max(config_.clearance - config_.lethal_radius, 1e-6);
    for (std::size_t i = 0; i < map.cost.size(); ++i)
    {
        double const d = nearest[i];
        if (d < config_.lethal_radius)
        {
            map.cost[i] = Costmap::kLethal;
        }
        else if (d < config_.clearance)
        {
            map.cost[i] = static_cast<float>(config_.clearance_weight * (1.0 - (d - config_.lethal_radius) / band));
        }
    }

    int const r = static_cast<int>(std::ceil(config_.lethal_radius / g.resolution));
    for (Point const p : { start, goal })
    {
        Cell const at = g.cell_of(p);
        for (int dy = -r; dy <= r; ++dy)
        {
            for (int dx = -r; dx <= r; ++dx)
            {
                Cell const c{ at.x + dx, at.y + dy };
                if (!g.contains(c) || distance(g.centre_of(c), p) > config_.lethal_radius)
                {
                    continue;
                }
                std::size_t const i = g.index(c);
                if (map.cost[i] >= Costmap::kLethal && g.cells[i] < config_.occupied)
                {
                    map.cost[i] = static_cast<float>(config_.clearance_weight);
                }
            }
        }
    }
    return map;
}

std::vector<Point> GridPlanner::plan(Costmap const& map, Point start, Point goal) const
{
    Grid const& g = *map.grid;
    int const reach = static_cast<int>(std::ceil(config_.lethal_radius / g.resolution)) + 1;
    Cell const from = nearest_passable(map, g.clamped(g.cell_of(start)), reach);
    Cell const to = nearest_passable(map, g.clamped(g.cell_of(goal)), reach);
    if (!passable(map, from) || !passable(map, to))
    {
        return {};
    }

    auto const cell_at = [&g](std::size_t i)
    {
        return Cell{ static_cast<int>(i % static_cast<std::size_t>(g.width)),
                     static_cast<int>(i / static_cast<std::size_t>(g.width)) };
    };
    auto const heuristic = [&](Cell c) { return distance(g.centre_of(c), g.centre_of(to)); };

    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::vector<double> best(g.cells.size(), kInfinity);
    std::vector<int> parent(g.cells.size(), -1);
    std::size_t const target = g.index(to);

    best[g.index(from)] = 0.0;
    open.push({ heuristic(from), g.index(from) });
    while (!open.empty())
    {
        auto const [f, i] = open.top();
        open.pop();
        if (i == target)
        {
            break;
        }
        Cell const c = cell_at(i);
        if (f > best[i] + heuristic(c) + 1e-9)
        {
            continue;  // superseded by a cheaper way in
        }
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                Cell const n{ c.x + dx, c.y + dy };
                if ((dx == 0 && dy == 0) || !passable(map, n))
                {
                    continue;
                }
                std::size_t const j = g.index(n);
                double const step = (dx != 0 && dy != 0 ? M_SQRT2 : 1.0) * g.resolution;
                double const cost = best[i] + step * (1.0 + map.cost[j]);
                if (cost < best[j])
                {
                    best[j] = cost;
                    parent[j] = static_cast<int>(i);
                    open.push({ cost + heuristic(n), j });
                }
            }
        }
    }
    if (best[target] == kInfinity)
    {
        return {};
    }

    std::vector<Cell> cells;
    for (int i = static_cast<int>(target); i >= 0; i = parent[static_cast<std::size_t>(i)])
    {
        cells.push_back(cell_at(static_cast<std::size_t>(i)));
    }
    std::reverse(cells.begin(), cells.end());

    // The real start and goal stand in for their cells' centres, unless either
    // had to move to a passable cell, in which case both are kept.
    std::vector<Point> path{ start };
    for (std::size_t k = 0; k < cells.size(); ++k)
    {
        bool const is_start = k == 0 && cells[k] == g.cell_of(start);
        bool const is_goal = k + 1 == cells.size() && cells[k] == g.cell_of(goal);
        if (!is_start && !is_goal)
        {
            path.push_back(g.centre_of(cells[k]));
        }
    }
    path.push_back(goal);
    return straighten(map, path);
}

}  // namespace prop_planner
