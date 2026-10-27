# Prop missions

`docking_mission` owns the docking state machine. It uses the
`prop_vision` docking library and publishes targets for the existing controller.
It does not start Gazebo, create a dock, or subscribe to ground truth.

Build and source the runtime packages:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select prop_vision prop_missions --symlink-install
source install/setup.bash
```

Once the sensor, localization, guidance, and thruster stacks are running:

```sh
ros2 launch prop_missions docking.launch.py
```

Or run the executable directly:

```sh
ros2 run prop_missions docking_mission
```

The launch defaults to real time. For a simulated clock, pass
`use_sim_time:=true`. The complete docking simulation already launches this
same executable; do not start a second mission alongside it.

Inputs: `/camera/image`, `/camera/camera_info`, `/lidar/points`, and
`odometry/filtered/global`. Outputs: `plan` (`nav_msgs/Path`) and `/dock/status`
(`std_msgs/String` containing JSON). Standard ROS remapping is supported.

The package move does not change the algorithm or establish hardware readiness.
The current prototype still uses temporary HSV color detection and assumes the
simulated mounting offsets (camera 0.1 m forward, LiDAR aligned with the boat),
small roll/pitch, one green and two red initially visible indicators, and a clear approach.
Clear-plan/watchdog stopping is not emergency braking or a retreat maneuver.
Calibrate and validate these assumptions before enabling vehicle control.
Camera-system integration is deferred pending the team discussion.

Tests, including recorded simulation data, live here and use normal installed
package imports:

```sh
colcon test --packages-select prop_missions --event-handlers console_direct+
```

Simulation-only scoring and randomized layouts remain in `prop_gazebo`.

Task 3 cleanup: the mission always seeks green; there is no requested-color `bay`
parameter. Repeated red indicator detections are preserved. Acquisition rejects
multiple green candidates. This is a docking prototype, not a full Core mission:
it now waits for an internal OCS START command, derives the bay number from
observations, and retries a docking event until receipt or timeout. A simulation-only
`mock_ocs` peer exercises this exchange. The real OCS adapter, official RoboCommand
MQTT/protobuf communication, heartbeats, and water delivery are outside this
vehicle-side package.

See [internal OCS interface](../../../../docs/docking-walkthrough/ocs-interface.md) for topics, messages, lifecycle, and simulation options.
