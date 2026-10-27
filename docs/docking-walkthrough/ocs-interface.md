# Docking vehicle-to-OCS interface

This is the team-internal ROS 2 contract between the docking mission on the boat
and the real OCS. It is not the RoboCommand MQTT/protobuf protocol. Only the OCS
connects to RoboCommand.

## Topics

All four topics use reliable `std_msgs/String` messages containing JSON.
Commands use transient-local durability with depth 1; the other topics are
volatile with depth 10.

| Topic | Direction | Purpose |
|---|---|---|
| `/dock/ocs/command` | OCS → boat | Start or cancel this docking attempt |
| `/dock/ocs/status` | boat → OCS | Current mission and receipt state |
| `/dock/ocs/result` | boat → OCS | Successfully detected docking bay |
| `/dock/ocs/receipt` | OCS → boat | Confirms the OCS accepted the result |

Every message contains:

```json
{"version":1,"kind":"...","vehicle_id":"prop","session_id":"...","run_id":"..."}
```

Identifiers are nonempty strings of at most 128 characters. The mission creates
a new random `session_id` each time it starts. The OCS must learn it from current
status rather than store it between mission processes. Before START, `run_id` is
`null`; the OCS assigns it in START.

START adds `command_id` and `action`:

```json
{"version":1,"kind":"command","vehicle_id":"prop","session_id":"CURRENT_SESSION","run_id":"CURRENT_RUN","command_id":"UNIQUE_COMMAND","action":"START"}
```

CANCEL has the same shape with `action: "CANCEL"`. The OCS should retry an
identical command, including its `command_id`, until status shows it took effect.
A matching CANCEL that arrives before START closes that session so a delayed
START cannot launch an aborted attempt.

After docking, the boat sends and retries an immutable result:

```json
{"version":1,"kind":"docking_result","vehicle_id":"prop","session_id":"CURRENT_SESSION","run_id":"CURRENT_RUN","event_id":"UNIQUE_EVENT","bay_number":2,"completed_at":123.4,"clock":"ros"}
```

The OCS validates and durably accepts that event, converts `bay_number` to the
official `RxReport.docking.bay_id`, and then returns:

```json
{"version":1,"kind":"receipt","vehicle_id":"prop","session_id":"CURRENT_SESSION","run_id":"CURRENT_RUN","event_id":"UNIQUE_EVENT","status":"RECEIVED"}
```

`RECEIVED` means the internal OCS accepted responsibility for the event. It does
not prove MQTT delivery, RoboCommand processing, or a score. Repeated identical
results must be deduplicated and acknowledged again. Conflicting contents under
the same event ID must be rejected.

## Lifecycle

1. The boat starts in `WAIT_START`, publishes no approach plan, and reports its
   fresh session ID every 0.5 ROS seconds.
2. The real OCS verifies official RunStart and selects Task 3, then sends START.
3. The mission enters `SEARCH`, detects the green bay, aligns, enters, and waits
   for the required stable pose and motion.
4. It enters `DOCKED`, latches one result, and retries it once per ROS second.
5. A matching receipt stops retries. Without one, receipt state becomes
   `TIMED_OUT` after the configured acknowledgement timeout.
6. CANCEL or local failure clears the navigation plan and stops pending retries.

The prototype supports one run per mission process. Restart it for another run.
Old sessions, runs, commands, and receipts are rejected. ROS timestamps are not
UTC and must not be copied into official `sent_at` fields.

The real OCS owns RunDeclaration, RunStart validation, 2 Hz heartbeats, actual
vehicle telemetry, per-vehicle report sequencing, official UTC timestamps,
MQTT delivery/reconnects, and operator status. The boat-side mission does not
implement active position hold while waiting, emergency stop, retreat, water
delivery, or official scoring confirmation.

## Simulation

The simulation-only `mock_ocs` starts the mission after three simulation seconds
and acknowledges results. It never connects to RoboCommand:

```sh
bash scripts/run_docking_sim.bash seed:=1 ocs_drop_receipts:=2
```

Set `mock_ocs:=false` when testing an external peer. Do not run the mock and the
real OCS command writer at the same time.
