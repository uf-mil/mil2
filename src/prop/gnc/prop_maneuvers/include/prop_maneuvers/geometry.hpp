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

#include <cmath>
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

    /// True when the clustering reported this bigger than any object on the course, so its size is clamped
    /// and its centre (the centroid of whatever merged) is untrustworthy. Still avoided, never adopted as the tracked
    /// object.
    bool merged{ false };
};

/// How a point sits relative to a straight leg.
struct Offset
{
    double perpendicular{ 0.0 };   ///< distance to the infinite line through a, b
    bool within_segment{ false };  ///< closest approach falls between a and b
};

/// Wrap to [-pi, pi], both ends inclusive (std::remainder can return exactly -pi).
double wrap_angle(double angle);

/// Radians to degrees, for log lines only; every angle in this package is radians.
constexpr double degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

/// Absolute direction from `from` to `to`.
double bearing(Point const &from, Point const &to);

/// Straight-line distance.
double distance(Point const &a, Point const &b);

/// True when `relative_angle` falls inside any of `blind_spots`.
/// An empty list means the lidar sees everywhere.
bool is_blind(double relative_angle, std::vector<BlindSpot> const &blind_spots);

/// The `index`-th corner of a ring of `legs` evenly spaced corners, at `radius` about `centre`.
/// The angle is absolute, measured from `entry_bearing` (object -> boat when the ring was entered), so
/// index 0 is the entry point and index `legs` is the same point one lap later. Stepping on from wherever
/// the boat actually reached carries every short corner round the lap and the lap never closes.
Point ring_corner(Point const &centre, double radius, double entry_bearing, int index, int legs,
                  bool counter_clockwise);

/// The point short of `target` by `standoff`, on the line from `start`.
/// Returns `start` unchanged when the target is already closer than the
/// standoff, so the boat never reverses to make room.
Point standoff_point(Point const &start, Point const &target, double standoff);

/// Where `p` sits relative to the leg `a` -> `b`.
/// `perpendicular` is the distance to the infinite line, not the segment: when `within_segment` is false
/// it understates the distance to the nearest endpoint, so do not read false as "far away".
Offset distance_to_segment(Point const &p, Point const &a, Point const &b);

/// True distance from `p` to the nearest point on the polyline through `path`, clamped at every segment end.
/// Unlike distance_to_segment this is the distance the boat actually keeps. An empty path returns
/// infinity; a one-point path returns the distance to that point.
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
/// Distances are hull to obstacle surface. `hull_half_width` is how far the hull reaches to the side of base_link.
///   - `min_gap` decides only whether a detour is needed; obstacles the hull passes no closer than this are left alone.
///   - `clearance` is how far out the detour swings.
/// Two waypoints, not one: a single one reaches full offset only abeam of the obstacle, so the path
/// bulges inward before it (1.05 m delivered against 1.25 m asked).
/// `achieved` reports the clearance the path really delivers, which can be below `clearance` (even
/// negative) when the goal itself lies inside it. Callers should log the shortfall rather than refuse.
Detour plan_detour(Point const &from, Point const &to, Blob const &obstacle, double hull_half_width, double min_gap,
                   double clearance);

/// How far `p` lies ahead of a boat at `boat` pointing along `travel_direction`; negative is behind.
/// Along-track only; sideways offset is ignored.
double along_track(Point const &boat, double travel_direction, Point const &p);

/// True while `obstacle` still lies ahead of a boat at `boat` travelling in `travel_direction`.
/// Answers "have I finished stepping around it?". A straddle is only correct as a whole: its first
/// waypoint carries the boat sideways, so a plan made half way finds nothing in the way and hands back
/// the straight line, which is why the committed pair is held until this returns false.
/// Along-track only, hull to surface: the obstacle is behind once its near surface clears the back of the
/// hull (`hull_behind` behind base_link), not when its centre draws level.
bool obstacle_ahead(Point const &boat, double travel_direction, Blob const &obstacle, double hull_behind);

/// True when nothing sits in the strip the hull would sweep reversing `distance` metres from `boat`
/// (`boat_direction` is its heading). The strip runs from base_link to `hull_behind + distance` behind it,
/// `hull_half_width` either side; a blob intrudes when its circle touches it.
/// Separate from Reverser, which stays dumb. Callers check before and during a reverse, on live data.
bool clear_behind(std::vector<Blob> const &blobs, Point const &boat, double boat_direction, double distance_back,
                  double hull_half_width, double hull_behind);

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

/// Pick the blob nearest `prediction`. Fails when the nearest is beyond `match_radius`, or when a second
/// blob is within `ambiguous_margin` of its distance; latching onto the wrong buoy is worse than stopping.
Match match_nearest(std::vector<Blob> const &blobs, Point const &prediction, double match_radius,
                    double ambiguous_margin);

/// Human-readable reason, for logging.
char const *describe(MatchFailure failure);

}  // namespace prop_maneuvers
