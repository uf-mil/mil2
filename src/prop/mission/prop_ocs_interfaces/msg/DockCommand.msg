# OCS -> boat: /dock/ocs/command
# All identifiers are nonempty; run_id is the official run ID as a decimal string.
string<=128 vehicle_id
string<=128 session_id
string<=128 run_id
# Reuse command_id when retrying the same command.
string<=128 command_id
string START=START
string CANCEL=CANCEL
string action
