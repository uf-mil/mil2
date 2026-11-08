# Prop missions

The boat's RobotX 2026 missions: `safe_passage` (Task 1) and `docking_mission`
(Task 3, without the spray).

## A run, start to finish

On the boat, in this order:

```sh
ros2 launch prop_bringup boat.launch.py            # gps_topic:=... if not /gps/fix
ros2 launch prop_vision cameras.launch.py model:=<weights.pt> publish_image:=true
ros2 launch prop_fusion fusion.launch.py           # labelled_objects
ros2 launch prop_missions safe_passage.launch.py   # waits for RunStart
ros2 launch prop_missions docking.launch.py        # waits for the OCS
prop-start-localization
start-controller                                   # holds position
```

On the OCS laptop, after `pip install -r requirements.txt` (the RoboCommand
messages need protobuf 7.36.1), with the course's RoboCommand address
(Alpha 192.168.65.2, Bravo .66.2, Charlie .67.2, Delta .68.2):

```sh
ros2 run ocs ocs --ros-args -p host:=192.168.65.2 -p "vehicles:=['USV']" \
    -p task2_tier:=NONE -p task4_tier:=NONE
```

Then `prop-auto` on the boat. Its heartbeat turns AUTO, RoboCommand sends
RunStart, the OCS passes it on, and Safe Passage runs. When the boat says Safe
Passage is over, the OCS starts docking. Declare task1_tier:=NONE to dock
straight away.

## Safe Passage

`safe_passage` says the task is SAFE_PASSAGE (that turns the lights on), waits
`light_delay`, clears what `beacon_node` saw in the dark, visits every buoy
whose light is unknown, circles the FLASHING_BLUE entry clockwise, passes red
to starboard and green to port on the way to the STEADY_BLUE exit, circles that
counter-clockwise, and says NONE. Settings, with what each does, are in
`config/safe_passage.yaml`; set `field_latlon` if the buoys are far from where
the boat starts.

The lights come from `prop_vision`'s `beacon_node`, on `/beacons`. Check it on
site against the real beacons with `cameras.launch.py publish_image:=true` and
`/beacons/front_camera/debug_image`: each box should sit on a beacon, grey when
dark and coloured when lit. Its window and colour thresholds are parameters of
`beacon_node` and `COLOR_RANGES` in `prop_vision/beacons.py`.

To try a change without the boat, `fake_field` stands in for the boat, planner
and beacons, and scores the run when it ends:

```sh
ros2 run prop_missions fake_field --ros-args -p layout:=zigzag
ros2 launch prop_missions safe_passage.launch.py start_now:=true
```

## Docking

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
(`std_msgs/String` containing JSON). Standard ROS remapping is supported, and
`docking.launch.py` remaps them to the boat's: the undistorted front camera
(`cameras.launch.py publish_image:=true`), `/velodyne_points`, and the local
estimate, because `trajectory_planner` follows plans in odom. Once started it
says the task is COORDINATED_LOGISTICS and switches the grid planner off.

The package move does not change the algorithm or establish hardware readiness.
The current prototype still uses temporary HSV color detection and assumes the
simulated mounting offsets (camera 0.1 m forward, LiDAR aligned with the boat),
small roll/pitch, one green and two red initially visible indicators, and a clear approach.
Clear-plan/watchdog stopping is not emergency braking or a retreat maneuver.
Calibrate and validate these assumptions before enabling vehicle control.

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
`mock_ocs` peer exercises this exchange in the sim, and the OCS (`src/ocs`) does
the same on the water. Water delivery is not done.

The internal exchange uses `/dock/ocs/command`, `/dock/ocs/status`,
`/dock/ocs/result`, and `/dock/ocs/receipt`, with versioned JSON envelopes in
`std_msgs/String`. See [the simulation README](../../simulation/README.md) for
the mock peer, receipt retries, and end-to-end tests.

The mission and simulation mock default to vehicle ID `USV`, matching the OCS
configuration. Internal run IDs carry the official numeric run ID as a string.
Bay numbers are 1–3 from left to right facing the dock, matching the official
handbook; the OCS forwards them unchanged as RoboCommand's DockingReport. It
sends START at RunStart when Task 1 is not attempted, and otherwise once the
boat's heartbeat says Safe Passage is over.
