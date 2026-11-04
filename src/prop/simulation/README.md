# Prop simulation

## Perception-driven docking

The docking exercise now uses a rendered RGB camera and VLP-16 point cloud to
choose a bay and compute navigation targets. The mission receives only
sensors, camera calibration, and estimated GPS/IMU odometry. Green selection is
fixed by the task rather than supplied as a requested bay.
It does **not** receive dock coordinates, color order, seed, or ground truth.

Build from the repository root with ROS Jazzy:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select prop_vision prop_missions prop_localization prop_controller prop_gazebo --symlink-install
```

Run (no additional sourcing needed):

```sh
bash scripts/run_docking_sim.bash seed:=1
```

The wrapper isolates boat packages from stale unrelated workspace installs,
selects Fast DDS without a Zenoh router, and uses ROS domain 75 with a dedicated
Gazebo partition. Run one instance at a time, or override both
`DOCKING_ROS_DOMAIN_ID` and `DOCKING_GZ_PARTITION` for independent instances.
The wrapper does not change the calling shell.

The `seed` changes dock translation, rotation, and the order of one green and
two red indicators reproducibly. The mission always selects green; `bay` is no
longer a launch argument.

The launch generates a temporary world and gives the fixture layout **only**
to `docking_evaluator.py`. The actual geometry dimensions are synthetic, not
competition measurements. The boat starts at the origin; the dock starts in
its field of view. There is no open-water search behavior yet.

Headless test, shutting down when the evaluator finishes:

```sh
bash scripts/run_docking_sim.bash seed:=3 \
  exit_on_result:=true gz_args:="-s --headless-rendering"
```

The fixture defaults to `real_time_factor:=0.5` to leave rendering time for both
sensors. Mission timing uses simulation seconds. Increase the factor only if
sensor streams remain timely. `timeout:=240.0` bounds the mission in simulation
time; a stopped clock still requires an external wall-clock timeout.

## Package ownership

- `prop_vision` owns the sensor geometry/color library under
  `src/prop/prop_vision/prop_vision` alongside the existing vision work.
- `prop_missions` owns `docking_mission`, its vehicle-facing launch, and tests
  under `src/prop/mission/prop_missions`.
- `prop_gazebo` owns only the simulated sensors, fixture, evaluator, and test launch.

Simulation runs the same `prop_missions` executable as the vehicle-facing
launch. Neither runtime package depends on Gazebo. The simulation wrapper and
command above remain unchanged. For the vehicle-facing interface and current
prototype limitations, see [the mission README](../mission/prop_missions/README.md).

## What the mission does

1. Segment red and green pixels using HSV thresholds, preserving separate
   same-color blobs. Require one green and two red indicators at acquisition. CameraInfo supplies the image projection.
2. Match image and LiDAR timestamps to estimated odometry. Project LiDAR returns
   into the map, filtering water and sky heights.
3. Fit wall and finger segments with RANSAC. Check that all three camera beacon
   rays intersect supported LiDAR geometry. Measure entrance inner edges from
   finger front caps, avoiding bias from an occluded side face.
4. Require three consistent bay estimates, then compute a staging point 2 m
   outside the observed entrance and a goal 1.25 m short of the observed back
   wall. No global target coordinates or color-position lookup are used.
5. Drive to staging at up to 0.3 m/s using the existing guidance and thruster
   manager. Require stable pose/motion and revalidate the target beacon against
   camera and LiDAR before entering.
6. Enter along the detected centerline and report `DOCKED` after a three-second
   estimated-pose dwell. The evaluator determines `PASS` independently.

Logs show `WAIT_START`, `STARTED`, `SEARCH`, `ALIGN`, `ENTER`, then `DOCKED`;
`RECEIVED` records the separate internal OCS receipt.
`/dock/status` also carries the mission state and detail as JSON in a String.
Camera topics are `/camera/image` and `/camera/camera_info`; LiDAR is
`/lidar/points`.

Stale sensors clear the plan (`PAUSED`); a continuous second of fresh data is
required before resuming. Sensor timestamp mismatch, LiDAR proximity inside
the clearance margin, or mission timeout aborts the mission. Clearing the plan
stops guidance publication and the existing command watchdog drops thrust; it
does not implement a controlled retreat or emergency braking maneuver.

## Independent scoring and tests

The evaluator alone subscribes to `/sim/ground_truth`. It checks the physical
bay corresponding to the green indicator, position, heading, low linear/angular
speed, and a three-second dwell after the mission reports `DOCKED`. Throughout
the run it checks dock clearance using a conservative circle enclosing the hull.
This is a geometric clearance check, not contact-sensor instrumentation. A PASS
certifies the measured dwell, not indefinite station keeping.

Offline tests include reordered/rotated/translated layouts, ambiguous or missing
color, missing geometry, stale-data stopping, and a recorded Gazebo snapshot
with a sparsely observed, partially occluded finger:

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select prop_missions --event-handlers console_direct+
```

Run the two positive cases and a hidden-beacon negative case sequentially:

```sh
python3 scripts/test_docking_sim.py
```

This runner uses independent ROS domains (100–102 by default, override with
`DOCKING_TEST_DOMAIN_BASE`) and unique Gazebo partitions. It saves logs in a
printed temporary directory, checks evaluator output rather than trusting the
launch exit code, and returns nonzero on failure. The negative case hides green
and must time out without ever commanding an approach:

```sh
bash scripts/run_docking_sim.bash seed:=4 hide_color:=green timeout:=30.0
```

## Scope and limitations

This is a perception-driven prototype for this simulated three-bay dock, not a
completed competition mission. It assumes one green and two red initially visible
indicators, approximately planar walls, calibrated camera/LiDAR mounting, and a
clear approach area. HSV thresholds are not validated against outdoor glare,
reflections, real beacon flashing, or distractors. A planar transform assumes
small boat roll and pitch. The test has no wind/current sweep or route planner
around arbitrary obstacles. It does not include tin handling or UAV delivery.
Real sensor calibration, official dock dimensions, hardware dynamics and
competition conditions still need validation before deploying to the vehicle.

The evaluator measures docking geometry; it does not award competition points or
validate RoboCommand reporting, which belongs to the real OCS.

## Internal OCS exchange

Simulation now starts `mock_ocs` alongside the mission. The mission waits in
WAIT_START until the mock sends START (default delay: 3 simulation seconds).
After docking it reports a sensor-derived bay number, retries the same event,
and records the matching receipt. The evaluator requires both physical docking
and the complete internal exchange before PASS. No RoboCommand connection exists.

Use `ocs_drop_receipts:=2` to exercise retries, `ocs_start_delay:=10.0` to extend
the wait, or `mock_ocs:=false` to wait for an external internal-protocol peer.
The regression runner drops the first two receipts in its positive cases.
See [the interface contract](../../../docs/docking-walkthrough/ocs-interface.md).
