# Ergonomic authoring for a body-relative move: translate by (x, y, z) meters in
# the body frame and rotate by (roll, pitch, yaw) degrees. The movement_manager
# converts this (degrees -> quaternion) and resolves it against the last goal, so
# no caller ever has to build a quaternion. For absolute moves use AbsoluteMove.
float64 x      # meters, body-frame forward
float64 y      # meters, body-frame left
float64 z      # meters, body-frame up
float64 roll   # degrees
float64 pitch  # degrees
float64 yaw    # degrees

# Precise mode: tighter tolerances + a longer timeout budget (see AbsoluteMove.precise).
# Default false = regular move.
bool precise

# Sender-assigned id, echoed back in MoveStatus (see AbsoluteMove.command_id).
uint32 command_id
