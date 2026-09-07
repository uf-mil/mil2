#pragma once

namespace prop_maneuvers
{

/// An angle range, relative to the front of the boat, that the lidar cannot
/// see through. Radians, left positive.
struct BlindSpot
{
    double from{ 0.0 };
    double to{ 0.0 };
};

}  // namespace prop_maneuvers
