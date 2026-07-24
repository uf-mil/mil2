# Telemetry from the movement_manager for the command it is currently executing.
# Published while a command is active, plus one final message on its terminal state.
std_msgs/Header header

# Echoes the AbsoluteMove/RelativeMove command_id so callers can correlate status
# with their request.
uint32 command_id

# Resolved absolute goal sent to the path planner (odom frame).
geometry_msgs/Pose goal

# Live error of the sub against the goal.
float64 dist_remaining  # meters
float64 ori_err_deg     # degrees

# True while the sub is within the tight (arrival_pos_tol, arrival_ori_tol_deg)
# tolerance this instant. Instantaneous telemetry; see `state` for the verdict.
bool at_goal

uint8 ACTIVE=0
uint8 REACHED=1
uint8 REACHED_LOOSE=2
uint8 TIMED_OUT=3
uint8 PREEMPTED=4
uint8 state

string message
