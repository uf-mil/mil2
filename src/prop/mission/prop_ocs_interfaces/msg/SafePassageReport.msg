# Boat -> OCS: /prop/safe_passage_report, sent again whenever it changes.
# The OCS copies it into RoboCommand's SafePassageReport.
# An entry or exit that is not found yet has a NaN latitude, and is left out.
geographic_msgs/GeoPoint entry_position
geographic_msgs/GeoPoint exit_position
Buoy[] buoys
