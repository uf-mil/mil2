#include "prop_maneuvers/geometry.hpp"

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

std::vector<Point> ring_corners(Point const &centre, Point const &from, double radius, int legs, bool counter_clockwise)
{
    std::vector<Point> corners;
    if (legs < 3)
    {
        return corners;
    }

    // Start on the line from the centre out through the boat, so a boat that
    // is already about the right distance out does not have to double back.
    double const start = bearing(centre, from);
    double const step = (counter_clockwise ? 1.0 : -1.0) * 2.0 * M_PI / static_cast<double>(legs);

    corners.reserve(static_cast<std::size_t>(legs));
    for (int i = 0; i < legs; ++i)
    {
        double const angle = start + step * static_cast<double>(i);
        corners.push_back(Point{ centre.x + radius * std::cos(angle), centre.y + radius * std::sin(angle) });
    }
    return corners;
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

std::optional<Point> detour_point(Point const &a, Point const &b, Point const &obstacle, double keep_out)
{
    Offset const offset = distance_to_segment(obstacle, a, b);
    if (!offset.within_segment || offset.perpendicular >= keep_out)
    {
        return std::nullopt;
    }

    double const dx = b.x - a.x;
    double const dy = b.y - a.y;
    // A zero-length leg (a == b) is already excluded above: distance_to_segment
    // forces within_segment = false in that case, so the check on offset
    // above already returned.
    double const length_squared = dx * dx + dy * dy;

    // Foot of the perpendicular from the obstacle onto the leg.
    double const along = ((obstacle.x - a.x) * dx + (obstacle.y - a.y) * dy) / length_squared;
    Point const foot{ a.x + dx * along, a.y + dy * along };

    // Unit vector pointing from the obstacle towards the leg. When the
    // obstacle sits exactly on the leg there is no such direction, so step to
    // the left of travel.
    double nx = foot.x - obstacle.x;
    double ny = foot.y - obstacle.y;
    double const length = std::hypot(nx, ny);
    if (length < 1e-9)
    {
        double const leg = std::sqrt(length_squared);
        nx = -dy / leg;
        ny = dx / leg;
    }
    else
    {
        nx /= length;
        ny /= length;
    }

    return Point{ obstacle.x + nx * keep_out, obstacle.y + ny * keep_out };
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
