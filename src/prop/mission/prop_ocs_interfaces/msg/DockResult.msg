# Boat -> OCS: /dock/ocs/result
# All identifiers must be nonempty. Retries preserve every field.
string<=128 vehicle_id
string<=128 session_id
string<=128 run_id
string<=128 event_id
# Valid values: 1, 2, 3. Copy directly to RxReport.docking.bay_id.
uint8 bay_number
# ROS clock time, including simulation time; never an official UTC timestamp.
builtin_interfaces/Time completed_at
