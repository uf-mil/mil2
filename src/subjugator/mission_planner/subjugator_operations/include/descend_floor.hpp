#pragma once

#include <algorithm>
#include <optional>

// Depth-floor arithmetic for DescendUntilDetected, kept in its own header (no
// BT/ROS includes) so it is unit-testable the same way detection_gate.hpp is.
namespace descend
{
// Next commanded depth for one downward step, or nullopt when the descent has
// run out of room and the node should give up.
//
// min_z is an ABSOLUTE floor in the odom frame, which this tree already treats
// as world z (LockOverTarget's table_z="-0.85" is a world tabletop height fed
// straight to an odom-frame comparison; depth is the axis where the two agree
// best, since it comes from the depth sensor).
//
// It exists because max_steps is NOT a depth bound, despite its comment saying
// it keeps us off the floor: 12 steps x 0.20 m is 2.4 m of descent, and from
// the `near` start at z=-0.35 that reaches -2.75, far below the -0.85 tabletop.
// Measured 2026-07-28: an actual run descended to -1.41 -- 0.56 m BELOW the
// tabletop -- from where a down camera cannot see the table at ANY lateral
// offset, so neither the descent nor a later spiral can ever succeed.
//
// The final step is clipped to the floor rather than overshooting it. A step
// with less than pos_tol of room left is refused instead of commanded, because
// the caller's own goal-reached test uses pos_tol: such a goal would read as
// already reached and the node would spin issuing it.
inline std::optional<double> next_descend_z(double cur_z, double step_m, double min_z, double pos_tol)
{
    double const next = std::max(cur_z - step_m, min_z);
    if (cur_z - next < pos_tol)
    {
        return std::nullopt;
    }
    return next;
}
}  // namespace descend
