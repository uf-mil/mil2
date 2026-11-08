#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "prop_planner/geometry.hpp"

namespace prop_planner
{

struct Cell
{
    int x{ 0 };
    int y{ 0 };
};

inline bool operator==(Cell a, Cell b)
{
    return a.x == b.x && a.y == b.y;
}

/// A raster over the map frame laid out like nav_msgs/OccupancyGrid: origin is
/// the outer corner of cell (0, 0) and cell (x, y) is stored at y * width + x.
struct Grid
{
    int width{ 0 };
    int height{ 0 };
    double resolution{ 0.25 };
    Point origin;
    std::vector<int8_t> cells;

    bool contains(Cell c) const
    {
        return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height;
    }

    std::size_t index(Cell c) const
    {
        return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(c.x);
    }

    Cell cell_of(Point p) const
    {
        return { static_cast<int>(std::floor((p.x - origin.x) / resolution)),
                 static_cast<int>(std::floor((p.y - origin.y) / resolution)) };
    }

    Point centre_of(Cell c) const
    {
        return { origin.x + (c.x + 0.5) * resolution, origin.y + (c.y + 0.5) * resolution };
    }

    Cell clamped(Cell c) const
    {
        return { std::clamp(c.x, 0, width - 1), std::clamp(c.y, 0, height - 1) };
    }
};

}  // namespace prop_planner
