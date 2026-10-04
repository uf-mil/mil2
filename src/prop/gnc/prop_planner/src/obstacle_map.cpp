#include "prop_planner/obstacle_map.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace prop_planner
{
namespace
{

/// Below this a capsule is a circle and its axis direction carries no meaning.
constexpr double kDegenerate = 1e-6;

/// Unit vector along the capsule's axis, or (0, 0) when it has no length.
Point axis_direction(Capsule const& capsule)
{
    double const len = length(capsule);
    if (len < kDegenerate)
    {
        return { 0.0, 0.0 };
    }
    return { (capsule.b.x - capsule.a.x) / len, (capsule.b.y - capsule.a.y) / len };
}

bool is_degenerate(Point direction)
{
    return std::hypot(direction.x, direction.y) < kDegenerate;
}

/// Where p falls along the axis through origin, in metres from it.
double project(Point p, Point origin, Point direction)
{
    return (p.x - origin.x) * direction.x + (p.y - origin.y) * direction.y;
}

Point along_axis(Point origin, Point direction, double t)
{
    return { origin.x + t * direction.x, origin.y + t * direction.y };
}

/// Move a toward b by gain, the blend used for every averaged quantity here.
double approach(double a, double b, double gain)
{
    return a + gain * (b - a);
}

}  // namespace

Capsule merge_capsules(Capsule stored, Capsule observation, double gain, double extent_deadband)
{
    // Pair the ends that correspond, or a reversed segment collapses the capsule.
    double const direct = distance(stored.a, observation.a) + distance(stored.b, observation.b);
    double const flipped = distance(stored.a, observation.b) + distance(stored.b, observation.a);
    if (flipped < direct)
    {
        std::swap(observation.a, observation.b);
    }

    // Two round entries leave the direction arbitrary; the extent is zero anyway.
    Point const stored_direction = axis_direction(stored);
    Point const observed_direction = axis_direction(observation);
    Point direction{ 1.0, 0.0 };
    if (is_degenerate(stored_direction) && !is_degenerate(observed_direction))
    {
        direction = observed_direction;
    }
    else if (!is_degenerate(stored_direction) && is_degenerate(observed_direction))
    {
        direction = stored_direction;
    }
    else if (!is_degenerate(stored_direction))
    {
        Point const blended{ approach(stored_direction.x, observed_direction.x, gain),
                             approach(stored_direction.y, observed_direction.y, gain) };
        double const norm = std::hypot(blended.x, blended.y);
        direction = norm < kDegenerate ? stored_direction : Point{ blended.x / norm, blended.y / norm };
    }
    Point const lateral{ -direction.y, direction.x };

    Point const stored_centre = midpoint(stored);
    Point const observed_centre = midpoint(observation);
    double const across = project(observed_centre, stored_centre, lateral);

    // Width is seen whole every frame, so sideways averages.
    Point const origin{ stored_centre.x + gain * across * lateral.x, stored_centre.y + gain * across * lateral.y };

    double const half = 0.5 * length(stored);
    double low = -half;
    double high = half;

    if (length(observation) > extent_deadband)
    {
        // A slice of the object: its ends accumulate rather than averaging,
        // or the capsule shrinks to whatever is in view and slides along.
        double const observed_low =
            std::min(project(observation.a, origin, direction), project(observation.b, origin, direction));
        double const observed_high =
            std::max(project(observation.a, origin, direction), project(observation.b, origin, direction));

        // Beat the deadband, or jitter alone stretches the entry every frame.
        if (observed_low < low - extent_deadband)
        {
            low = approach(low, observed_low, gain);
        }
        if (observed_high > high + extent_deadband)
        {
            high = approach(high, observed_high, gain);
        }
    }
    else
    {
        // Position, not extent: slide the shape and keep its length.
        double const along = project(observed_centre, origin, direction);
        low += gain * along;
        high += gain * along;
    }

    return { along_axis(origin, direction, low), along_axis(origin, direction, high),
             approach(stored.radius, observation.radius, gain) };
}

void ObstacleMap::observe(Capsule observation)
{
    observation = clamped(observation);

    int const index = nearest(observation);
    if (index < 0)
    {
        if (obstacles_.size() >= config_.capacity)
        {
            evict_least_seen();
        }
        // Below the deadband, length is not evidence, so do not record it -
        // nothing later shortens an entry, only slides it.
        if (length(observation) <= config_.extent_deadband)
        {
            Point const centre = midpoint(observation);
            observation = Capsule{ centre, centre, observation.radius };
        }
        obstacles_.push_back(Obstacle{ observation, 1, 0, true });
        return;
    }

    Obstacle& entry = obstacles_[static_cast<std::size_t>(index)];
    entry.shape = clamped(merge_capsules(entry.shape, observation, config_.position_gain, config_.extent_deadband));
    ++entry.hits;
    entry.seen = true;
}

void ObstacleMap::forget_unseen(Point sensor)
{
    for (Obstacle& entry : obstacles_)
    {
        if (entry.seen)
        {
            entry.seen = false;
            entry.misses = 0;
        }
        else if (distance_to_segment(sensor, entry.shape.a, entry.shape.b) <= config_.verify_range)
        {
            ++entry.misses;
        }
    }

    obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(),
                                    [this](Obstacle const& o) { return o.misses > config_.max_misses; }),
                     obstacles_.end());
}

std::vector<Obstacle> ObstacleMap::confirmed() const
{
    std::vector<Obstacle> out;
    out.reserve(obstacles_.size());
    std::copy_if(obstacles_.begin(), obstacles_.end(), std::back_inserter(out),
                 [this](Obstacle const& o) { return is_confirmed(o); });
    return out;
}

int ObstacleMap::nearest(Capsule const& observation) const
{
    // Axis to axis, not centre to centre: the two ends of a dock share an axis
    // while their centres are far apart. Linear scan; there are tens of these.
    int best = -1;
    double best_distance = config_.merge_distance;

    for (std::size_t i = 0; i < obstacles_.size(); ++i)
    {
        Capsule const& stored = obstacles_[i].shape;
        double const gap = segment_distance(observation.a, observation.b, stored.a, stored.b);
        if (gap < best_distance)
        {
            best_distance = gap;
            best = static_cast<int>(i);
        }
    }
    return best;
}

Capsule ObstacleMap::clamped(Capsule capsule) const
{
    capsule.radius = std::min(capsule.radius, config_.max_radius);

    double const len = length(capsule);
    if (len <= config_.max_length || len < kDegenerate)
    {
        return capsule;
    }

    // Trim about the centre so a bad association cannot grow without bound.
    Point const centre = midpoint(capsule);
    Point const direction = axis_direction(capsule);
    double const half = 0.5 * config_.max_length;
    return { along_axis(centre, direction, -half), along_axis(centre, direction, half), capsule.radius };
}

void ObstacleMap::evict_least_seen()
{
    // Fewest observations: most likely to have been noise.
    auto const victim = std::min_element(obstacles_.begin(), obstacles_.end(),
                                         [](Obstacle const& a, Obstacle const& b) { return a.hits < b.hits; });
    obstacles_.erase(victim);
}

}  // namespace prop_planner
