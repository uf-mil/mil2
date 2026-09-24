# prop_planner

On-demand 2D A* planning, separate from mission selection and path execution.
A mission chooses destinations and their order; this node computes a route
between a start and a goal using the latest occupancy grid. A reachable goal
is required: a general-purpose planner cannot guarantee it can go anywhere.

## Interfaces

- Service: `~/plan` (normally `/prop_planner/plan`), `nav_msgs/srv/GetPlan`.
  Supply `start` and `goal` as `PoseStamped`. An empty start frame requests the
  latest odometry pose; an explicit start works without odometry.
  Set `tolerance: 0.0`; nonzero tolerances are currently rejected.
- Map input: `/prop_planner/mock_map`, `nav_msgs/msg/OccupancyGrid`, configurable
  with `global_map_topic`. Reliable, transient-local QoS, depth 1; the publisher
  must offer transient-local durability. This is a placeholder interface, not
  the map branch's `obstacle_map` MarkerArray interface.
- Position input: `/odometry/filtered/global`, `nav_msgs/msg/Odometry`, configurable
  with `odom_topic`. Sensor-data QoS accepts best-effort and reliable publishers.
  Position logging remains throttled to once per second.
- Result: `response.plan`, a `nav_msgs/msg/Path`. Empty means planning failed;
  the node logs the reason. GetPlan has no separate status/message fields.

The service returns a path without publishing movement commands. A mission
caller can inspect it and then publish the accepted path on `plan` with reliable,
transient-local QoS, depth 1, for Prop guidance. Avoid competing with the existing
fixed-waypoint mission publisher. Requests do not initiate continuous replanning;
map changes require another call. Subscriptions continue between calls, while a
synchronous search temporarily occupies the node's single-threaded executor.

## Algorithm and limits

Four-connected A* searches free grid cells (occupancy 0–49); unknown and occupied
cells are blocked. The result includes the exact endpoints and traversed cell
centers, preserving the requested final orientation. Every pose shares the path's
frame and timestamp. Endpoint-to-center connections stay inside endpoint cells.

Inputs must share the map frame; no TF conversion is performed. Only axis-aligned
maps with a unit identity rotation are accepted. Invalid metadata, non-finite or
out-of-bounds positions, blocked endpoints, and unreachable goals return an empty
path. Grids are limited to one million cells to bound search memory and work.
This is planar planning; z is carried in poses but is not collision-checked.
The map must already include vessel clearance. There is no obstacle inflation,
turn-radius constraint, dynamic-obstacle handling, smoothing, or input-age check.
A collision-free grid route is not a guarantee that guidance's actual trajectory
will clear obstacles. This is a planning prototype for synthetic maps.

## Build and try with a synthetic map

In Bash, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select prop_planner
source install/setup.bash
ros2 run prop_planner prop_planner_node
```

In another sourced terminal:

```bash
ros2 run prop_planner mock_map.py
```

The mock publisher retains a 20 × 20 grid with 1 m cells in `map`, starting at
(0, 0), with a wall at x = 10–11 m and a passage beyond y = 15 m. It remains
running so late subscribers can receive the map. In a third sourced terminal:

```bash
ros2 service call /prop_planner/plan nav_msgs/srv/GetPlan "{
  start: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 2.5}, orientation: {w: 1.0}}},
  goal: {header: {frame_id: map}, pose: {position: {x: 17.5, y: 2.5}, orientation: {w: 1.0}}},
  tolerance: 0.0
}"
```

The returned path detours through the passage. This explicit-start example needs
no localization process. To use the boat's latest position, omit `start` and
ensure odometry has arrived in `map`.

## Relationship to existing nodes

- Prop `prop_controller/src/mission.cpp` publishes a configured waypoint path
  on `plan`. It is currently a publisher node, not a planning service.
- Prop guidance consumes `plan` and odometry, then publishes `cmd_vel`.
- Subjugator's PID controller consumes odometry and desired poses and publishes
  `cmd_wrench`. Its `~/enable` and `~/reset` services manage controller state;
  the control loop itself operates continuously through topics.
- Subjugator localization uses `robot_localization`'s EKF to fuse IMU, DVL velocity,
  and depth into estimated state. Its launch file exposes set-pose, enable,
  toggle, and a custom reset service. The reset wrapper asynchronously requests
  a zero pose; its response does not confirm completion of that downstream call.
- Subjugator's current `PathPlanner.cpp` receives goals through a topic and
  publishes paths; having a `PathPlan.srv` definition elsewhere does not make
  that implementation a service server.

This planner follows the persistent-node/service-callback pattern from those
management services, but its operation is to compute and return a route.

## Tests

```bash
ROS_DOMAIN_ID=87 colcon test --packages-select prop_planner --event-handlers console_direct+
colcon test-result --test-result-base build/prop_planner --verbose
```

Tests launch the actual executable and cover odometry logging, missing map and
odometry, explicit and implicit starts, wall avoidance, path headers and final
orientation, frame mismatch, unsupported tolerance, non-finite/out-of-bounds
goals, unknown goals, unreachable routes, and malformed grids. Use an unused ROS
domain to isolate testing from boat software.
