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

/// True distance from `p` to the nearest point ON the polyline through
/// `path`, clamped at every segment end.
///
/// Unlike distance_to_segment, which reports the perpendicular to an infinite
/// line, this is the distance the boat actually keeps. That difference is the
/// whole reason the old single-waypoint detour under-delivered: the waypoint
/// was correct and the path was not.
///
/// An empty path returns infinity; a one-point path returns the distance to
/// that point.
double distance_to_polyline(Point const &p, std::vector<Point> const &path);

/// What, if anything, the boat should do about an obstacle on its leg.
enum class DetourNeed
{
    None,             ///< nothing blocking; drive straight
    Straddle,         ///< go around, via `before` then `after`
    TooCloseToSwing,  ///< too close to make the clearance; back off first
};

/// A planned way past one obstacle.
struct Detour
{
    DetourNeed need{ DetourNeed::None };
    Point before;            ///< only meaningful when need == Straddle
    Point after;             ///< only meaningful when need == Straddle
    double achieved{ 0.0 };  ///< hull-to-surface clearance the driven path delivers
};

/// Plan a way past `obstacle` while travelling `from` -> `to`.
///
/// Every distance here is measured from the HULL to the obstacle's SURFACE,
/// so the numbers mean what a person would picture. `hull_half_width` is how
/// far the hull reaches to the side of base_link, which is what odometry
/// reports and what this function's points are expressed in.
///
///   - `min_gap` decides only WHETHER a detour is needed: an obstacle the
///     hull would pass no closer than this is left alone. Wide enough
///     obstacles are avoided, gates are driven through.
///   - `clearance` is how far out the detour actually swings.
///
/// Two waypoints, not one. A single waypoint reaches full sideways offset
/// only as the boat draws level with the obstacle, so the path bulges inward
/// before it -- measured at 1.05 m against 1.25 m asked for. Two waypoints
/// finish the sideways move before the obstacle and hold it past.
///
/// `achieved` reports the clearance the composed path really delivers. It can
/// be less than `clearance`, and can even be negative, when the goal itself
/// lies inside the clearance: the path ends at the goal, so no routing can
/// recover that. Callers should log the shortfall rather than refuse.
Detour plan_detour(Point const &from, Point const &to, Blob const &obstacle, double hull_half_width, double min_gap,
                   double clearance);

/// Why a match attempt did not produce a blob.
enum class MatchFailure
{
    None,       ///< matched
    NoBlobs,    ///< nothing in range at all
    TooFar,     ///< nearest blob is further than the limit from the prediction
    Ambiguous,  ///< two blobs are equally plausible; refuse to guess
};

/// The result of trying to match one blob to a predicted position.
struct Match
{
    bool ok{ false };
    Blob blob;
    MatchFailure failure{ MatchFailure::NoBlobs };
};

/// Pick the blob nearest `prediction`.
///
/// Fails when the nearest is further than `match_radius`, and fails when a
/// second blob is within `ambiguous_margin` of the nearest one's distance --
/// silently latching onto the wrong buoy is worse than stopping.
Match match_nearest(std::vector<Blob> const &blobs, Point const &prediction, double match_radius,
                    double ambiguous_margin);

/// Human-readable reason, for logging.
char const *describe(MatchFailure failure);

}  // namespace prop_maneuvers
