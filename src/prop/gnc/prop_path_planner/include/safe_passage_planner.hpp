#pragma once

#include <vector>

namespace prop_controller
{

struct Point
{
    double x = 0.0;
    double y = 0.0;
};

enum class BuoyColor
{
    BLACK,       // an obstacle, pass on either side
    RED,         // keep on the boat's starboard (right) side
    GREEN,       // keep on the boat's port (left) side
    FLASH_BLUE,  // the ENTRY buoy, circle clockwise
    SOLID_BLUE   // the EXIT buoy, circle counterclockwise
};

struct Buoy
{
    Point position;
    BuoyColor color = BuoyColor::BLACK;
};

// A Buoy's base is a square of 15.25" PVC legs, so its widest point
// (a corner) is about 0.27 m from center, the pool-noodle flotation adds a
// little more. ~0.3 m is a estimation radius until it's measured
// off a real buoy.
constexpr double kApproxBuoyRadius = 0.3;

// to ensure we do not crash into a buoy.
constexpr double kRequiredClearance = 1.0;

constexpr double kBuoyStandoff = kApproxBuoyRadius + kRequiredClearance;  // ~1.3 m

struct PlannerParams
{
    double pass_offset = kBuoyStandoff;    // how far from a red/green buoy's center to aim, meters
    double circle_radius = kBuoyStandoff;  // radius of the entry/exit circling maneuver, meters
    double safety_margin = kBuoyStandoff;  // how close we're allowed to get to a black/obstacle buoy
    int circle_segments = 16;              // how many waypoints to use to approximate a circle
                                           // 16 because there's 16 points on a unit circle idk
};

std::vector<Point> planSafePassage(std::vector<Buoy> const& buoys, Point boat_start, PlannerParams const& params = {});

}  // namespace prop_controller
