#pragma once

#include <vector>

#include "prop_planner/grid.hpp"

namespace prop_planner
{

/// What crossing each cell costs on top of its length: 0 in open water, rising
/// toward an obstacle, kLethal inside the hull's margin around one.
struct Costmap
{
    static constexpr float kLethal = 1000.0f;

    Grid const* grid{ nullptr };
    std::vector<float> cost;
};

/// Length of a route weighted by the cells it crosses; a leg through the margin
/// costs kLethal per metre, so a blocked route is always the dearer one.
double route_cost(Costmap const& map, std::vector<Point> const& route);

/// A* over the evidence grid, then straightened.
///
/// Every cell within lethal_radius of a solid one is off limits, and cells out
/// to clearance cost extra, falling off linearly, so a route centres itself
/// between a gate's buoys rather than skimming one. The cell path is then cut
/// down to the few corners that matter: each straight leg replaces the cells
/// between its ends only where it costs no more than they did.
class GridPlanner
{
  public:
    struct Config
    {
        int occupied{ 60 };              ///< evidence at or above this is solid
        double lethal_radius{ 0.6 };     ///< m, half the beam plus a margin
        double clearance{ 2.0 };         ///< m, beyond which an obstacle costs nothing
        double clearance_weight{ 2.0 };  ///< extra cost per metre at the margin's edge
    };

    explicit GridPlanner(Config config) : config_(config)
    {
    }

    /// Costs for the grid as it stands. The margin is waived around start and
    /// goal, since the boat is where it is and a goal beside a buoy is still
    /// the goal; solid cells stay lethal everywhere.
    Costmap costmap(Grid const& evidence, Point start, Point goal) const;

    /// Waypoints from start to goal inclusive, or empty when nothing connects them.
    std::vector<Point> plan(Costmap const& map, Point start, Point goal) const;

  private:
    Config config_;
};

}  // namespace prop_planner
