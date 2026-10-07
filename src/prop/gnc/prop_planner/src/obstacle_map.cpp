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
    // A segment can arrive either way round. Pair the ends that actually
    // correspond, or the blend below pulls both toward the middle and the
    // capsule collapses.
    double const direct = distance(stored.a, observation.a) + distance(stored.b, observation.b);
    double const flipped = distance(stored.a, observation.b) + distance(stored.b, observation.a);
    if (flipped < direct)
    {
        std::swap(observation.a, observation.b);
    }

    // Blend the direction, falling back on whichever of the two has one. Two
    // round entries leave it arbitrary, which costs nothing because the extent
    // below comes out at zero either way.
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

    // Everything below is measured from the stored centre, along the blended
    // axis and across it.
    Point const stored_centre = midpoint(stored);
    Point const observed_centre = midpoint(observation);
    double const across = project(observed_centre, stored_centre, lateral);

    // Sideways always averages: the observation sees the whole width of the
    // object every frame, so there is nothing to accumulate.
    Point const origin{ stored_centre.x + gain * across * lateral.x, stored_centre.y + gain * across * lateral.y };

    double const half = 0.5 * length(stored);
    double low = -half;
    double high = half;

    if (length(observation) > extent_deadband)
    {
        // An observation with real length is a slice of the object, so what it
        // says about the ends accumulates rather than averaging - each view
        // reveals a different part. Averaging here would pull the capsule down
        // to whichever slice is currently visible and let it slide along the
        // object as the boat drives past.
        double const observed_low =
            std::min(project(observation.a, origin, direction), project(observation.b, origin, direction));
        double const observed_high =
            std::max(project(observation.a, origin, direction), project(observation.b, origin, direction));

        // Reaching past a stored end has to beat the deadband, or observation
        // jitter alone would stretch an entry a little further every frame.
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
        // A round observation carries position, not extent. The whole shape
        // slides along the axis toward it and keeps the length it had.
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
        // Below the deadband a length is not evidence of extent, so it is not
        // worth recording either. Without this a round buoy keeps whatever
        // spurious length its very first frame happened to fit, forever -
        // every later observation only slides it, never shortens it.
        if (length(observation) <= config_.extent_deadband)
        {
            Point const centre = midpoint(observation);
            observation = Capsule{ centre, centre, observation.radius };
        }
        obstacles_.push_back(Obstacle{ observation, 1 });
        return;
    }

    Obstacle& entry = obstacles_[static_cast<std::size_t>(index)];
    entry.shape = clamped(merge_capsules(entry.shape, observation, config_.position_gain, config_.extent_deadband));
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

int ObstacleMap::nearest(Capsule const& observation) const
{
    // Between axes rather than between centres: see one end of a dock and then
    // the other and the centres are far apart, though the two observations are
    // of the same object and their axes lie almost on top of each other.
    //
    // Linear, because the map holds tens of entries and a detection arrives
    // ten times a second. A spatial index here would cost more to read than it
    // could ever save.
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

    // Trim evenly about the centre, so a bad association that welded two
    // objects together cannot keep growing without bound.
    Point const centre = midpoint(capsule);
    Point const direction = axis_direction(capsule);
    double const half = 0.5 * config_.max_length;
    return { along_axis(centre, direction, -half), along_axis(centre, direction, half), capsule.radius };
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
