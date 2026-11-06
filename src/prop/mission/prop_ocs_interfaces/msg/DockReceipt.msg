# OCS -> boat: /dock/ocs/receipt
# Match all four identifiers from the accepted result; all must be nonempty.
string<=128 vehicle_id
string<=128 session_id
string<=128 run_id
string<=128 event_id
# Send only after durable storage/queueing; this is not scoring confirmation.
string RECEIVED=RECEIVED
string status
