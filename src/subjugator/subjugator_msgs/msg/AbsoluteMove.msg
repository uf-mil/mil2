# A request for the movement_manager to drive to an absolute pose in the odom
# frame. The manager resolves it to a single goal_pose for the path planner and
# reports progress on move_status. For body-relative moves use RelativeMove.

# Absolute goal pose in the odom frame.
geometry_msgs/Pose target

# Which axes hold the sub's current pose instead of taking them from `target`.
# OR the bits together (e.g. HOLD_X|HOLD_Y for a depth-only move).
uint8 HOLD_X=1
uint8 HOLD_Y=2
uint8 HOLD_Z=4
uint8 HOLD_ORIENTATION=8
uint8 hold

# Precise mode: aim for tighter tolerances with a longer timeout budget. On timeout
# the grace window relaxes to the regular tolerances (not the wide loose ones a
# regular move uses). Default false = regular move.
bool precise

# Sender-assigned id, echoed back in MoveStatus so a caller can tell which command
# a status refers to and ignore a stale "arrived" left over from a prior move.
uint32 command_id
