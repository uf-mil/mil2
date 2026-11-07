# prop_planner

Continuous coordinate planning using a persistent `ObstacleMap` and
visibility-graph A* from `carlos-path-plannning`.

## Interfaces

- Service `~/active` (normally `/prop_planner/active`): `std_srvs/srv/Trigger`.
  Query only; `success` reports the `active` parameter and `message` is
  `active` or `inactive`. The default is false. While inactive, odometry, goal, and
  perception callbacks return immediately, and planning/publication stop.
  The status and toggle services remain available. Set it from launch or
  at runtime. It does not guarantee a valid route or fresh inputs.
- Service `~/toggle_active`: `std_srvs/srv/Trigger`. Each call flips the active
  parameter: false → true → false. Like `~/active`, the response `success`
  carries the resulting flag (false means inactive, not a failed toggle).
  No goal, odometry, or perception input is needed to query or toggle it.
- Input `goal_pose`: `geometry_msgs/msg/PoseStamped`, reliable/volatile depth 10.
  The latest goal replaces the previous goal, including invalid goals. Poses
  are transformed into `map_frame` at their timestamps; zero requests latest TF.
- Output `/path`: `nav_msgs/msg/Path`, reliable/volatile depth 10, published at
  10 Hz while active using a wall timer. Each tick replans from the latest odometry to the
  latest goal using confirmed obstacles. Missing inputs or planning failures
  publish an empty path instead of retaining an old route.
- Input `tracked_markers`: `visualization_msgs/msg/MarkerArray`, reliable/volatile
  depth 10, configurable through `tracks_topic`. ADD/CUBE detections are
  transformed individually and converted to capsules. DELETEALL clears the
  tracker's visualization, not persistent obstacle memory.
- Input `/odometry/filtered/global`: `nav_msgs/msg/Odometry`, sensor-data QoS,
  configurable through `odom_topic`.

Paths contain the odometry start, route waypoints, and transformed goal. Every
pose shares the path header. A zero goal quaternion means no requested final
heading; otherwise use a unit quaternion. Topic names can be remapped through
ROS arguments. Toggle active before sending inputs. Messages received while inactive are
ignored; previously accepted inputs and the persistent map are retained.
While inactive, a reminder is logged every two seconds. Activation logs once;
routine position logs are disabled and planning diagnostics use DEBUG. The former `~/plan` GetPlan service has been removed.

## Map and planning behavior

The reused map defaults confirm obstacles after three associated observations,
merge observations within 2 m of their axes, and retain up to 256 entries. At
capacity the least-seen entry is evicted. Objects do not expire just because they
leave the sensor view. These map settings currently use the core defaults.

Parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `active` | `false` | Enable input processing, planning, and publication |
| `map_frame` | `map` | Common planning frame |
| `odom_topic` | `/odometry/filtered/global` | Estimated boat pose |
| `tracks_topic` | `tracked_markers` | Perception detections |
| `inflation` | `2.0` | Clearance added to obstacle radii, metres |
| `corners_per_obstacle` | `8` | Candidates around each end cap; range 4–64 |

A* connects candidate points with collision-free straight segments, allowing
arbitrary directions. String pulling removes unnecessary waypoints. Search is
shortest on the sampled graph, not guaranteed globally shortest in continuous
space. The planner rejects start/goal positions inside confirmed inflated
obstacles before invoking the core, which otherwise ignores endpoint overlaps.

Missing inputs, invalid goals, missing TF, and unreachable goals return an
empty path while active; the reason is available at DEBUG log level.

No perception messages yet means planning fails. An empty scan is valid and
allows straight-line planning. Receipt does not establish complete coverage:
unobserved space and unconfirmed detections are not blocked. There are no map
bounds, freshness checks, moving-obstacle prediction,
curvature constraints, or vertical collision checks. A synchronous timer callback
plans against one snapshot of confirmed obstacles. The default map/global
odometry pairing matches this branch's guidance; keep frames consistent when
connecting the path output to guidance.

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
centred at (10, 0). Supply odometry from localization, then publish a goal:

```bash
ros2 service call /prop_planner/toggle_active std_srvs/srv/Trigger '{}'
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: map},
  pose: {position: {x: 20.0, y: 0.0}, orientation: {w: 1.0}}
}"
ros2 service call /prop_planner/active std_srvs/srv/Trigger '{}'
ros2 topic echo /path
```

For real detections, stop the mock publisher and supply the perception stack's
`tracked_markers`, TF, and localization. In simulation, set
`--ros-args -p use_sim_time:=true` on the node.

## Tests

```bash
ROS_DOMAIN_ID=87 colcon test --packages-select prop_planner --event-handlers console_direct+
colcon test-result --test-result-base build/prop_planner --verbose
```

C++ tests cover geometry, map memory, and visibility planning. ROS tests cover
missing inputs, empty scans, confirmation, segment clearance, map persistence,
blocked endpoints, invalid inputs, odometry starts, TF for detections/start/goal,
and preservation of unspecified final heading. Use an unused ROS domain.
