#pragma once

#include <cmath>

namespace prop_planner
{

struct Point
{
    double x{ 0.0 };
    double y{ 0.0 };
};

inline double distance(Point a, Point b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

// A segment thickened by a radius: a rectangle with round caps, sometimes
// called a stadium.
//
// A circle is the case where the segment has no length, so nothing downstream
// needs to special case round objects - a buoy is a capsule whose two ends
// coincide. What the shape buys over a circle is elongation: a 20 m dock
// enclosed by a circle blocks 317 m2 of open water, and the same dock as a
// capsule blocks 39 m2, against a true footprint of 40 m2.
//
// It also keeps inflation free. Growing a capsule by a safety margin is
// radius + margin and nothing else, where growing a polygon means a Minkowski
// sum with a disc - offset edges, arcs at every vertex, and spikes wherever
// the hull comes to a point.
struct Capsule
{
    Point a;
    Point b;
    double radius{ 0.0 };
};

inline double length(Capsule const& capsule)
{
    return distance(capsule.a, capsule.b);
}

inline Point midpoint(Capsule const& capsule)
{
    return { 0.5 * (capsule.a.x + capsule.b.x), 0.5 * (capsule.a.y + capsule.b.y) };
}

/// A capsule grown by a margin is the same capsule with a fatter radius.
inline Capsule inflate(Capsule const& capsule, double margin)
{
    return { capsule.a, capsule.b, capsule.radius + margin };
}

/// Closest approach of the segment ab to the point p.
double distance_to_segment(Point p, Point a, Point b);

/// Whether ab and cd properly cross. Touching and collinear cases report
/// false; callers measuring distance get ~0 from the endpoint tests anyway.
bool segments_cross(Point a, Point b, Point c, Point d);

/// Closest approach between the segments ab and cd, zero where they cross.
double segment_distance(Point a, Point b, Point c, Point d);

/// Whether the segment ab enters the capsule.
inline bool hits(Capsule const& capsule, Point a, Point b)
{
    return segment_distance(a, b, capsule.a, capsule.b) < capsule.radius;
}

/// Whether p is inside the capsule.
inline bool contains(Capsule const& capsule, Point p)
{
    return distance_to_segment(p, capsule.a, capsule.b) < capsule.radius;
}

}  // namespace prop_planner
