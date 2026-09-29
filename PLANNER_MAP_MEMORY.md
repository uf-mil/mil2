# Planner map integration note

## Confirmed map-branch interfaces

`src/prop/gnc/prop_planner/src/planner.cpp` already creates a publisher on
the relative topic `plan` with message type `nav_msgs/msg/Path` and reliable,
transient-local QoS, depth 1. `Planner::publish_plan()` publishes the computed
route with a timestamp and `map_frame` (default `map`) on the path and every
pose. The final pose preserves the goal orientation.

The planner waits for odometry on `odometry/filtered/global` and a goal on
`goal_pose` (`geometry_msgs/msg/PoseStamped`). It replans at `rate` (default
1 Hz) and publishes nonempty routes when the waypoint count changes or a
waypoint moves more than `replan_threshold` (default 0.5 m). A new goal
forces a fresh plan. If no route exists, it holds the last published plan.

`src/prop/gnc/prop_controller/src/guidance.cpp` already subscribes to `plan`
using `nav_msgs/msg/Path` and matching QoS. Avoid repeatedly publishing an
unchanged path because each receipt resets guidance's leg tracking.

The actual map output in this branch is `obstacle_map`, a
`visualization_msgs/msg/MarkerArray` published with reliable, volatile QoS,
depth 1. It is NOT an `OccupancyGrid` on `/map`. It begins with DELETEALL,
then CYLINDER obstacle markers in `confirmed` or `pending` namespaces and
TEXT_VIEW_FACING hit-count labels. Cylinder diameters include the planner's
inflation margin. The internal map receives `tracked_markers` detections
and transforms them into `map_frame`.

When returning to the prop planner branch, use these confirmed interfaces
to choose the appropriate subscription. The earlier `/map` OccupancyGrid
subscription below was only provisional and does not match this publisher.
The `plan` publishing implementation already existed when inspected; no
source changes were needed for that request. Runtime publishing was not
tested during this inspection.

## Earlier provisional subscription

The user is switching to the map branch. Preserve the map interface details
needed to connect the `prop_planner` node to the map publisher.

The planner implementation from the preceding branch used:
- Message type: `nav_msgs/msg/OccupancyGrid`.
- Topic: `/map`, configurable via the `global_map_topic` parameter.
- Subscription QoS: reliable, transient-local, depth 1.
- Callback: retain the latest `OccupancyGrid::ConstSharedPtr` in
  `last_global_map_` without copying the grid; null means no map received yet.
- Source: `src/prop/gnc/prop_planner/src/prop_planner_node.cpp`.

The topic and message type above were implementation defaults because the
previous branch did not define a map publisher interface. Confirm the actual
publisher topic, message type, and QoS on the map branch before integrating;
adjust the planner subscription as needed. Transient-local subscription QoS
requires a transient-local publisher.

The planner package built successfully after that change. At the time this
note was saved, the planner source path was absent from the current checkout.
