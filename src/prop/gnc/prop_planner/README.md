# prop_planner

On-demand coordinate planning using Carlos's persistent `ObstacleMap` and
visibility-graph A* from `carlos-path-plannning`. The geometry, map, search, and
C++ tests are reused. This replaces the earlier OccupancyGrid/four-connected
prototype; the ROS wrapper keeps the `nav_msgs/srv/GetPlan` service contract.

## Interfaces

- Service `~/plan` (normally `/prop_planner/plan`): supply stamped start and goal
  poses with `tolerance: 0`. An empty start frame uses the latest odometry.
  Poses are transformed into `map_frame` at their timestamps; zero timestamp
  requests the latest TF. Missing transforms return an empty path.
- Input `tracked_markers`: `visualization_msgs/msg/MarkerArray`, reliable/volatile
  depth 10, configurable through `tracks_topic`. ADD/CUBE detections are
  transformed individually and converted to capsules. DELETEALL clears the
  tracker's visualization, not persistent obstacle memory.
- Input `/odometry/filtered/global`: `nav_msgs/msg/Odometry`, sensor-data QoS,
  configurable through `odom_topic`. Explicit-start requests need no odometry.
  Position logging remains throttled to once per second.
- Response `plan`: `nav_msgs/msg/Path`, with the exact start, route waypoints,
  and transformed goal pose. Every pose shares the path header. A zero goal
  quaternion means no requested final heading; otherwise use a unit quaternion.

The node only returns paths. It does not subscribe to `goal_pose`, publish `plan`,
or command movement. A calling node must select the path and publish it to
guidance with reliable, transient-local QoS, depth 1. Do not run competing
automatic path publishers on that same topic.

## Map and planning behavior

The reused map defaults confirm obstacles after three associated observations,
merge observations within 2 m of their axes, and retain up to 256 entries. At
capacity the least-seen entry is evicted. Objects do not expire just because they
leave the sensor view. These map settings currently use the core defaults.

Parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `map_frame` | `map` | Common planning frame |
| `odom_topic` | `/odometry/filtered/global` | Estimated boat pose |
| `tracks_topic` | `tracked_markers` | Perception detections |
| `inflation` | `2.0` | Clearance added to obstacle radii, metres |
| `corners_per_obstacle` | `8` | Candidates around each end cap; range 4–64 |

A* connects candidate points with collision-free straight segments, allowing
arbitrary directions. String pulling removes unnecessary waypoints. Search is
shortest on the sampled graph, not guaranteed globally shortest in continuous
space. The service rejects start/goal positions inside confirmed inflated
obstacles before invoking the core, which otherwise ignores endpoint overlaps.

Missing inputs, invalid requests, missing TF, and unreachable goals return an
empty path with the reason logged. GetPlan has no separate status field.

No perception messages yet means planning fails. An empty scan is valid and
allows straight-line planning. Receipt does not establish complete coverage:
unobserved space and unconfirmed detections are not blocked. There are no map
bounds, automatic replanning, freshness checks, moving-obstacle prediction,
curvature constraints, or vertical collision checks. A synchronous callback
plans against one snapshot of confirmed obstacles. The default map/global
odometry pairing matches this branch's guidance; keep frames consistent when
connecting a caller to guidance.

## Build and try

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

The historical name `mock_map.py` is retained, but it now publishes repeated
tracked CUBE markers, not OccupancyGrid. The synthetic obstacle is 2 m by 8 m,
centred at (10, 0). After at least three observations, request a route around it:

```bash
ros2 service call /prop_planner/plan nav_msgs/srv/GetPlan "{
  start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}},
  goal: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 0.0}, orientation: {w: 1.0}}},
  tolerance: 0.0
}"
```

For real detections, stop the mock publisher and supply the perception stack's
`tracked_markers`, TF, and localization. In simulation, set
`--ros-args -p use_sim_time:=true` on the service. This package change does not
import Carlos's Gazebo launch or start his automatic topic-based planner.

## Tests

```bash
ROS_DOMAIN_ID=87 colcon test --packages-select prop_planner --event-handlers console_direct+
colcon test-result --test-result-base build/prop_planner --verbose
```

C++ tests cover geometry, map memory, and visibility planning. ROS tests cover
missing inputs, empty scans, confirmation, segment clearance, map persistence,
blocked endpoints, invalid inputs, odometry starts, TF for detections/start/goal,
and preservation of unspecified final heading. Use an unused ROS domain.
