# Boat -> OCS: /dock/ocs/status
string<=128 vehicle_id
string<=128 session_id
# Empty before a run is bound (replaces JSON null).
string<=128 run_id
# Empty until a result exists (replaces JSON null).
string<=128 event_id
string WAIT_START=WAIT_START
string SEARCH=SEARCH
string ALIGN=ALIGN
string ENTER=ENTER
string DOCKED=DOCKED
string FAIL=FAIL
string state
string NONE=NONE
string PENDING=PENDING
string RECEIVED=RECEIVED
string TIMED_OUT=TIMED_OUT
string CANCELLED=CANCELLED
string receipt
bool paused
# ROS clock time, including simulation time; never an official UTC timestamp.
builtin_interfaces/Time sent_at
