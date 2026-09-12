#include "prop_maneuvers/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace prop_maneuvers
{
namespace
{
/// Wrap to [0, 2*pi), which makes "is this angle inside that range" a single
/// comparison even when the range crosses directly-behind.
double positive_wrap(double angle)
{
    double const two_pi = 2.0 * M_PI;
    double wrapped = std::fmod(angle, two_pi);
    if (wrapped < 0.0)
    {
        wrapped += two_pi;
    }
    return wrapped;
}
}  // namespace

double wrap_angle(double angle)
{
    return std::remainder(angle, 2.0 * M_PI);
}

double bearing(Point const &from, Point const &to)
{
    return std::atan2(to.y - from.y, to.x - from.x);
}

double distance(Point const &a, Point const &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool is_blind(double relative_angle, std::vector<BlindSpot> const &blind_spots)
{
    for (auto const &spot : blind_spots)
    {
        double const span = positive_wrap(spot.to - spot.from);
        double const offset = positive_wrap(relative_angle - spot.from);
        if (span > 0.0 && offset <= span)
        {
            return true;
        }
    }
    return false;
}

Point standoff_point(Point const &start, Point const &target, double standoff)
{
    double const span = distance(start, target);
    if (span <= standoff || span == 0.0)
    {
        // Already inside the standoff. Stay put rather than reversing.
        return start;
    }
    double const scale = (span - standoff) / span;
    return Point{ start.x + (target.x - start.x) * scale, start.y + (target.y - start.y) * scale };
}

Offset distance_to_segment(Point const &p, Point const &a, Point const &b)
{
    double const dx = b.x - a.x;
    double const dy = b.y - a.y;
    double const length_squared = dx * dx + dy * dy;

    if (length_squared == 0.0)
    {
        return Offset{ distance(p, a), false };
    }

    double const along = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared;
    // Perpendicular distance to the infinite line, via the 2D cross product.
    double const across = std::abs((p.x - a.x) * dy - (p.y - a.y) * dx) / std::sqrt(length_squared);

    return Offset{ across, along >= 0.0 && along <= 1.0 };
}

double distance_to_polyline(Point const &p, std::vector<Point> const &path)
{
    if (path.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    if (path.size() == 1)
    {
        return distance(p, path.front());
    }

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i)
    {
        Point const &a = path[i];
        Point const &b = path[i + 1];
        double const dx = b.x - a.x;
        double const dy = b.y - a.y;
        double const length_squared = dx * dx + dy * dy;

        double t = 0.0;
        if (length_squared > 0.0)
        {
            t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared;
            t = std::clamp(t, 0.0, 1.0);  // the clamp is the point of this function
        }
        best = std::min(best, distance(p, Point{ a.x + dx * t, a.y + dy * t }));
    }
    return best;
}

Detour plan_detour(Point const &from, Point const &to, Blob const &obstacle, double hull_half_width, double min_gap,
                   double clearance)
{
    // Everything the caller gave us is hull-to-surface. Convert once, here, to
    // centre-to-centre, which is what the geometry below works in.
    double const trigger_radius = obstacle.radius + hull_half_width + min_gap;
    double const swing_radius = obstacle.radius + hull_half_width + clearance;

    Detour result;

    Offset const offset = distance_to_segment(obstacle.centre, from, to);
    if (!offset.within_segment || offset.perpendicular >= trigger_radius)
    {
        result.need = DetourNeed::None;
        return result;
    }

    // No path can be further from the obstacle than the point it starts at, so
    // if the boat is already inside the swing there is nothing to plan yet.
    if (distance(from, obstacle.centre) < swing_radius)
    {
        result.need = DetourNeed::TooCloseToSwing;
        return result;
    }

    double const dx = to.x - from.x;
    double const dy = to.y - from.y;
    // A zero-length leg cannot reach here: distance_to_segment forces
    // within_segment = false for one, and that returned above.
    double const length_squared = dx * dx + dy * dy;
    double const length = std::sqrt(length_squared);

    // Along-track unit vector.
    double const tx = dx / length;
    double const ty = dy / length;

    // Foot of the perpendicular from the obstacle onto the leg.
    double const along = ((obstacle.centre.x - from.x) * dx + (obstacle.centre.y - from.y) * dy) / length_squared;
    Point const foot{ from.x + dx * along, from.y + dy * along };

    // Unit vector from the obstacle towards the leg -- the clear side. When
    // the obstacle sits exactly on the leg there is no such direction, so step
    // to the left of travel.
    double nx = foot.x - obstacle.centre.x;
    double ny = foot.y - obstacle.centre.y;
    double const across = std::hypot(nx, ny);
    if (across < 1e-9)
    {
        nx = -ty;
        ny = tx;
    }
    else
    {
        nx /= across;
        ny /= across;
    }

    // Spacing equals the sideways offset: the tightest that meets the
    // clearance across the swept range, and a 45 degree entry leg.
    double const spacing = swing_radius;

    // Both waypoints are held ON the leg, between the boat and the goal.
    //
    // The perpendicular offset contributes nothing along-track, so before and
    // after sit at the obstacle's along-track position minus and plus spacing.
    // Neither trigger gate bounds that. within_segment only says the obstacle
    // is somewhere between the ends, and TooCloseToSwing only rejects a boat
    // already inside the swing -- so an obstacle nearly ABEAM of the boat
    // passes both and puts `before` behind it, while one nearly AT the goal
    // puts `after` past it.
    //
    // A waypoint behind the boat is the worse of the two: guidance restarts at
    // waypoint 0 on every hand-over and scales its forward speed by the cosine
    // of the bearing error, so a point astern reads as zero speed and a full
    // rate turn -- the boat spins on the spot and backtracks for a point it
    // has already passed. A waypoint past the goal simply carries the bow
    // nearer the object than the standoff promises.
    //
    // Clamping rather than refusing: a squeezed straddle still beats a
    // straight line, and `achieved` below is measured on the points actually
    // produced, so the caller's under-delivery warning stays honest.
    double const foot_along = along * length;
    double const before_along = std::max(0.0, foot_along - spacing);
    double const after_along = std::min(length, foot_along + spacing);

    result.need = DetourNeed::Straddle;
    result.before = Point{ obstacle.centre.x + nx * swing_radius + tx * (before_along - foot_along),
                           obstacle.centre.y + ny * swing_radius + ty * (before_along - foot_along) };
    result.after = Point{ obstacle.centre.x + nx * swing_radius + tx * (after_along - foot_along),
                          obstacle.centre.y + ny * swing_radius + ty * (after_along - foot_along) };

    std::vector<Point> const driven{ from, result.before, result.after, to };
    result.achieved = distance_to_polyline(obstacle.centre, driven) - obstacle.radius - hull_half_width;
    return result;
}

double along_track(Point const &boat, double travel_direction, Point const &p)
{
    double const tx = std::cos(travel_direction);
    double const ty = std::sin(travel_direction);
    return (p.x - boat.x) * tx + (p.y - boat.y) * ty;
}

bool obstacle_ahead(Point const &boat, double travel_direction, Blob const &obstacle, double hull_behind)
{
    // Behind only once the obstacle's near SURFACE has cleared the back of the
    // hull, not merely once its centre draws level with base_link.
    return along_track(boat, travel_direction, obstacle.centre) > -(obstacle.radius + hull_behind);
}

bool clear_behind(std::vector<Blob> const &blobs, Point const &boat, double boat_direction, double distance_back,
                  double hull_half_width, double hull_behind)
{
    // Backwards along the hull, and to its left. Working in the boat's own
    // frame turns "is it in the strip" into a point-to-rectangle question.
    double const bx = -std::cos(boat_direction);
    double const by = -std::sin(boat_direction);
    double const lx = -std::sin(boat_direction);
    double const ly = std::cos(boat_direction);

    double const strip_length = hull_behind + distance_back;

    for (auto const &blob : blobs)
    {
        double const ox = blob.centre.x - boat.x;
        double const oy = blob.centre.y - boat.y;

        double const back = ox * bx + oy * by;  // positive is behind the boat
        double const side = ox * lx + oy * ly;  // positive is to its left

        // Distance from the blob's centre to the rectangle [0, strip_length]
        // x [-half_width, +half_width]. Zero when the centre is inside it.
        double const dback = std::max({ 0.0, -back, back - strip_length });
        double const dside = std::max(0.0, std::abs(side) - hull_half_width);

        if (std::hypot(dback, dside) < blob.radius)
        {
            return false;
        }
    }
    return true;
}

Match match_nearest(std::vector<Blob> const &blobs, Point const &prediction, double match_radius,
                    double ambiguous_margin)
{
    Match result;

    if (blobs.empty())
    {
        result.failure = MatchFailure::NoBlobs;
        return result;
    }

    // Find the nearest and the runner-up in one pass.
    std::size_t nearest = 0;
    double nearest_range = distance(blobs[0].centre, prediction);
    double runner_up_range = std::numeric_limits<double>::max();

    for (std::size_t i = 1; i < blobs.size(); ++i)
    {
        double const range = distance(blobs[i].centre, prediction);
        if (range < nearest_range)
        {
            runner_up_range = nearest_range;
            nearest_range = range;
            nearest = i;
        }
        else if (range < runner_up_range)
        {
            runner_up_range = range;
        }
    }

    if (nearest_range > match_radius)
    {
        result.failure = MatchFailure::TooFar;
        return result;
    }

    // Two blobs about equally close to where we expected one. Latching onto
    // the wrong buoy would send the boat round the wrong thing, so stop.
    if (runner_up_range - nearest_range < ambiguous_margin)
    {
        result.failure = MatchFailure::Ambiguous;
        return result;
    }

    result.ok = true;
    result.blob = blobs[nearest];
    result.failure = MatchFailure::None;
    return result;
}

char const *describe(MatchFailure failure)
{
    switch (failure)
    {
        case MatchFailure::None:
            return "matched";
        case MatchFailure::NoBlobs:
            return "nothing in view";
        case MatchFailure::TooFar:
            return "nearest blob is too far from where the buoy was expected";
        case MatchFailure::Ambiguous:
            return "two blobs are equally plausible; refusing to guess";
    }
    return "unknown";
}

}  // namespace prop_maneuvers
