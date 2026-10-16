#include "prop_planner/geometry.hpp"

#include <algorithm>

namespace prop_planner
{
namespace
{

/// Sign of the cross product (b - a) x (c - a): which side of ab the point c
/// falls on.
double side(Point a, Point b, Point c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

}  // namespace

double distance_to_segment(Point p, Point a, Point b)
{
    double const dx = b.x - a.x;
    double const dy = b.y - a.y;
    double const length_squared = dx * dx + dy * dy;

    // A segment of no length is just its endpoint, which is also the whole of
    // the round-obstacle case.
    if (length_squared < 1e-12)
    {
        return distance(p, a);
    }

    // Project p onto the infinite line through a and b, then clamp the
    // projection back onto the segment itself.
    double const t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

bool segments_cross(Point a, Point b, Point c, Point d)
{
    double const d1 = side(a, b, c);
    double const d2 = side(a, b, d);
    double const d3 = side(c, d, a);
    double const d4 = side(c, d, b);

    return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
}

double segment_distance(Point a, Point b, Point c, Point d)
{
    if (segments_cross(a, b, c, d))
    {
        return 0.0;
    }

    // Two segments that do not cross have their closest approach at an
    // endpoint of one of them, so four point-to-segment tests cover it.
    return std::min({ distance_to_segment(a, c, d), distance_to_segment(b, c, d), distance_to_segment(c, a, b),
                      distance_to_segment(d, a, b) });
}

}  // namespace prop_planner
