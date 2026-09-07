/**
 * @file geometry.hpp
 * @brief Pure geometry for the boat maneuvers. No ROS, no I/O, no state.
 *
 * Angles are radians, measured with LEFT positive, and wrapped to [-pi, pi]
 * -- both ends inclusive; std::remainder can and does return exactly -pi.
 * A "direction" is absolute, in map coordinates. A "relative angle" is
 * measured from the front of the boat.
 */

#pragma once

#include <optional>
#include <vector>

namespace prop_maneuvers
{

/// A point in map coordinates, metres.
struct Point
{
    double x{ 0.0 };
    double y{ 0.0 };
};

/// An angle range, relative to the front of the boat, that the lidar cannot
/// see through. Radians, left positive. May wrap across directly-behind.
struct BlindSpot
{
    double from{ 0.0 };
    double to{ 0.0 };
};

/// A blob seen by the clustering, expressed in map coordinates.
struct Blob
{
    Point centre;
    double radius{ 0.0 };
};

/// How a point sits relative to a straight leg.
struct Offset
{
    double perpendicular{ 0.0 };   ///< distance to the infinite line through a, b
    bool within_segment{ false };  ///< closest approach falls between a and b
};

/// Wrap to [-pi, pi], both ends inclusive (std::remainder can return exactly
/// -pi; that is not a bug to "round" the other way).
double wrap_angle(double angle);

/// Absolute direction from `from` to `to`.
double bearing(Point const &from, Point const &to);

/// Straight-line distance.
double distance(Point const &a, Point const &b);

/// True when `relative_angle` falls inside any of `blind_spots`.
/// An empty list means the lidar sees everywhere.
bool is_blind(double relative_angle, std::vector<BlindSpot> const &blind_spots);

/// The corner points of the shape driven around `centre` at `radius`.
///
/// The FIRST corner lies on the line from `centre` out through `from`, so a
/// boat already near that line enters the shape without doubling back. The
/// remaining corners follow at even spacing in the chosen direction. Driving
/// corner 0 -> 1 -> ... -> n-1 -> 0 goes all the way round.
///
/// `legs` must be at least 3; smaller values return an empty list.
///
/// `from == centre` is a degenerate ray with no defined direction; `atan2(0,
/// 0)` is 0 in C++, so this silently starts the ring due east. That is an
/// arbitrary answer to an undefined question, not a considered default --
/// callers should not rely on it, and should avoid asking the question.
std::vector<Point> ring_corners(Point const &centre, Point const &from, double radius, int legs,
                                bool counter_clockwise);

/// The point short of `target` by `standoff`, on the line from `start`.
/// Returns `start` unchanged when the target is already closer than the
/// standoff, so the boat never reverses to make room.
Point standoff_point(Point const &start, Point const &target, double standoff);

/// Where `p` sits relative to the leg `a` -> `b`.
///
/// `perpendicular` is the distance to the INFINITE LINE, not the segment: for
/// a `p` whose closest approach falls just past either end (`within_segment
/// == false`), the true distance to the nearest endpoint is understated. A
/// point at (10.5, 0) against the leg (0,0) -> (10,0) is only 0.5 m past the
/// end, but this returns the perpendicular distance to the line (0) with
/// `within_segment == false` -- callers must not read a false
/// `within_segment` as "far away".
Offset distance_to_segment(Point const &p, Point const &a, Point const &b);

/// A single waypoint that steps around `obstacle` when travelling `a` -> `b`,
/// leaving `keep_out` metres between the waypoint and the obstacle centre.
///
/// Returns nothing when the obstacle does not block the leg -- either it is
/// further than `keep_out` from the line, or its closest approach falls
/// outside the segment. When the obstacle sits exactly on the line, the
/// detour goes to the left of travel.
std::optional<Point> detour_point(Point const &a, Point const &b, Point const &obstacle, double keep_out);

}  // namespace prop_maneuvers
