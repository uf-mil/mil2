#include "prop_planner/evidence_grid.hpp"

#include <algorithm>
#include <cmath>

namespace prop_planner
{

EvidenceGrid::EvidenceGrid(Config config) : config_(config)
{
    int const side = std::max(1, static_cast<int>(std::lround(config.size / config.resolution)));
    grid_.width = grid_.height = side;
    grid_.resolution = config.resolution;
    grid_.cells.assign(static_cast<std::size_t>(side) * static_cast<std::size_t>(side), 0);
    hit_this_frame_.assign(grid_.cells.size(), 0);
}

void EvidenceGrid::follow(Point boat)
{
    double const half = 0.5 * config_.size;
    Point const centre{ grid_.origin.x + half, grid_.origin.y + half };

    // Nothing of the old window overlaps a boat this far off, so start again
    // around it. Localization can hand out a wild position for a moment as it
    // comes up, and sliding a window that far is not arithmetic worth doing.
    if (!placed_ || distance(boat, centre) > config_.size)
    {
        grid_.origin = { boat.x - half, boat.y - half };
        clear();
        placed_ = true;
        return;
    }

    if (distance(boat, centre) < 0.5 * half)
    {
        return;
    }
    shift(static_cast<int>(std::lround((boat.x - centre.x) / grid_.resolution)),
          static_cast<int>(std::lround((boat.y - centre.y) / grid_.resolution)));
}

void EvidenceGrid::shift(int dx, int dy)
{
    std::vector<int8_t> moved(grid_.cells.size(), 0);
    for (int y = 0; y < grid_.height; ++y)
    {
        for (int x = 0; x < grid_.width; ++x)
        {
            Cell const from{ x + dx, y + dy };
            if (grid_.contains(from))
            {
                moved[grid_.index({ x, y })] = grid_.cells[grid_.index(from)];
            }
        }
    }
    grid_.cells.swap(moved);
    grid_.origin.x += dx * grid_.resolution;
    grid_.origin.y += dy * grid_.resolution;
}

void EvidenceGrid::integrate(std::vector<Point> const& hits, Point sensor)
{
    // A cell counts once per frame however many points land in it: evidence
    // builds over time, not with point density.
    for (Point const& hit : hits)
    {
        Cell const c = grid_.cell_of(hit);
        if (!grid_.contains(c))
        {
            continue;
        }
        std::size_t const i = grid_.index(c);
        if (hit_this_frame_[i])
        {
            continue;
        }
        hit_this_frame_[i] = 1;
        touched_.push_back(i);
        grid_.cells[i] = static_cast<int8_t>(std::min(100, grid_.cells[i] + config_.hit_gain));
    }

    int const reach = static_cast<int>(std::ceil(config_.verify_range / grid_.resolution));
    Cell const at = grid_.cell_of(sensor);
    for (int y = at.y - reach; y <= at.y + reach; ++y)
    {
        for (int x = at.x - reach; x <= at.x + reach; ++x)
        {
            Cell const c{ x, y };
            if (!grid_.contains(c) || distance(grid_.centre_of(c), sensor) > config_.verify_range)
            {
                continue;
            }
            std::size_t const i = grid_.index(c);
            if (!hit_this_frame_[i])
            {
                grid_.cells[i] = static_cast<int8_t>(std::max(0, grid_.cells[i] - config_.miss_loss));
            }
        }
    }

    for (std::size_t i : touched_)
    {
        hit_this_frame_[i] = 0;
    }
    touched_.clear();
}

void EvidenceGrid::clear()
{
    std::fill(grid_.cells.begin(), grid_.cells.end(), 0);
}

}  // namespace prop_planner
