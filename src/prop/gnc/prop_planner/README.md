# prop_planner

First planner milestone: subscribe to `nav_msgs/msg/Odometry` and print the
boat's estimated x, y, z position in meters, along with its coordinate frame.
Output is throttled to at most once per second and only occurs when a message
arrives. Coordinates are relative to the incoming `header.frame_id`, not
latitude/longitude. This node does not yet generate paths or command movement.

Each callback stores the complete latest odometry message in `last_odom_` and
sets `has_odom_` to true. Future planning must check this flag before using the
stored message, since a valid position of (0, 0, 0) must not be confused with
having received no odometry. The flag indicates receipt, not data freshness.
The global-map subscription receives `nav_msgs/msg/OccupancyGrid` on `/map`
(configurable with `global_map_topic`). It uses reliable, transient-local QoS
to receive the retained map from a compatible publisher. The callback retains
the latest message in `last_global_map_` without copying the grid; the pointer
is null until the first map arrives.

The default topic is `/odometry/filtered/global`, published by
`prop_localization/launch/localization.launch.py` in the `map` frame.

## Build and run

From the workspace root, in a Bash terminal:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select prop_planner
source install/setup.bash
ros2 run prop_planner prop_planner_node
```

To select another odometry topic at startup:

```bash
ros2 run prop_planner prop_planner_node --ros-args -p odom_topic:=/odometry/filtered/local
```

## Try without a boat

Run the node on a separate test topic:

```bash
ros2 run prop_planner prop_planner_node --ros-args -p odom_topic:=/prop_planner/test_odom
```

In another terminal with ROS sourced, publish a sample message:

```bash
ros2 topic pub --once /prop_planner/test_odom nav_msgs/msg/Odometry \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 4.5, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

The node should print:

```text
Boat position in frame 'map': x=4.50 m, y=2.00 m, z=0.00 m
```

## Automated tests

After building, run in a terminal with ROS sourced:

```bash
ROS_DOMAIN_ID=87 colcon test --packages-select prop_planner --event-handlers console_direct+
colcon test-result --test-result-base build/prop_planner --verbose
```

Use an unused ROS domain to keep tests separate from running boat software.
The tests in `test/test_prop_planner.py` start the actual C++ executable and
publish sample odometry using Python. They check the default topic, a custom
topic with a best-effort publisher, no position output before a message,
acceptance of zero coordinates, and updated position/frame output after a
later message. They test observable behavior, not private state directly.
