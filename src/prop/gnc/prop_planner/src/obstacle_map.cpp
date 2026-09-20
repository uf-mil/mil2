#include "prop_planner/obstacle_map.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace prop_planner
{

void ObstacleMap::observe(double x, double y, double radius)
{
    radius = std::min(radius, config_.max_radius);

    int const index = nearest(x, y);
    if (index < 0)
    {
        if (obstacles_.size() >= config_.capacity)
        {
            evict_least_seen();
        }
        obstacles_.push_back(Obstacle{ x, y, radius, 1 });
        return;
    }

    // Fold the observation in rather than replacing with it. A cluster centroid
    // walks toward whichever face of the buoy the lidar can currently see, so
    // any single frame is a worse estimate than the running average of the
    // frames before it.
    Obstacle& entry = obstacles_[static_cast<std::size_t>(index)];
    double const gain = config_.position_gain;
    entry.x += gain * (x - entry.x);
    entry.y += gain * (y - entry.y);
    entry.radius += gain * (radius - entry.radius);
    ++entry.hits;
}

std::vector<Obstacle> ObstacleMap::confirmed() const
{
    std::vector<Obstacle> out;
    out.reserve(obstacles_.size());
    std::copy_if(obstacles_.begin(), obstacles_.end(), std::back_inserter(out),
                 [this](Obstacle const& o) { return is_confirmed(o); });
    return out;
}

int ObstacleMap::nearest(double x, double y) const
{
    // Linear, because the map holds tens of entries and a detection arrives ten
    // times a second. A spatial index here would cost more to read than it
    // could ever save.
    int best = -1;
    double best_distance = config_.merge_distance;

    for (std::size_t i = 0; i < obstacles_.size(); ++i)
    {
        double const distance = std::hypot(x - obstacles_[i].x, y - obstacles_[i].y);
        if (distance < best_distance)
        {
            best_distance = distance;
            best = static_cast<int>(i);
        }
    }
    return best;
}

void ObstacleMap::evict_least_seen()
{
    // The entry with the fewest observations is the one most likely to have
    // been noise in the first place.
    auto const victim = std::min_element(obstacles_.begin(), obstacles_.end(),
                                         [](Obstacle const& a, Obstacle const& b) { return a.hits < b.hits; });
    obstacles_.erase(victim);
}

}  // namespace prop_planner
