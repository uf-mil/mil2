# prop_mission_planner

Behavior-tree missions for the boat. Same idea as the sub's `mission_planner`:
each mission is an XML tree, picked by name, run by one node that exits with
SUCCESS or FAILURE.

## Before you run

The mission planner drives an already-running stack; it does not start one.
Before `bmp`, make sure:

- the boat or sim stack is up: localization is publishing
  `odometry/filtered/global`, pcd clustering is publishing `cluster_markers`,
  and guidance is consuming `plan`;
- `rmw_zenohd` is running (`pgrep -x rmw_zenohd`);
- `install/setup.bash` is sourced in the shell you run `bmp` from.

The node loads the mission first, so a wrong name fails at once. It then
waits (logging every 5 s) for a fresh position estimate and a clock before
starting the mission, so a missing piece above shows up as a stall, not a
crash. There is no default mission: without `-p mission:=<name>` it lists the
known ones and exits.

## Running a mission

    bmp BuoyTour --sim        # in simulation (use_sim_time)
    bmp BuoyTour              # on the boat

`bmp` (in `scripts/setup.bash`) passes `prop_maneuvers/config/maneuvers.yaml`
as the parameter file, so the maneuvers use the same tuning as the standalone
programs. Tab completion lists the installed missions.

Watch it live in Groot2 (port 1667, `-p groot_port:=0` to turn it off; Groot2
also uses the next port up, so 1 to 65534). If the
port is already busy, or the value given is out of range, the node logs a
warning and the mission runs without Groot2. Every transition is also printed
to the console.

Ctrl-C releases guidance and zeroes `cmd_vel`, repeats the zero for half a
second so a last guidance command cannot outlive it, then exits: the boat
coasts, it does not hold position. A second Ctrl-C kills the process immediately,
without sending those stop messages -- use it only if the first one hangs.

## Writing one

Add a file under `missions/`, give it a `<BehaviorTree ID="...">`, and add an
`<include>` for it in `missions/prop_missions.xml`. `test_missions` then checks
that it loads.

    <StaticObject   point="-4;-4" ref="{buoy}"/>
    <RosTimeout msec="300000">
        <CircleObject target="{buoy}" direction="clockwise"/>
    </RosTimeout>

| Node | Ports |
|---|---|
| `StaticObject` | `point="x;y"` (map frame), `ref="{name}"` |
| `FaceObject` | `target="{ref}"` or `in_front="true"`; `lock_timeout` (s, default 10); `ref` (out) |
| `ApproachObject` | as FaceObject, plus `standoff` (m); rejects a negative standoff |
| `CircleObject` | as FaceObject, plus `direction` (required: `clockwise` / `counter_clockwise`), `radius`, `legs`; rejects radius <= 0, legs < 3, and a ring too tight to clear the buoy |
| `RosTimeout` / `RosDelay` | `msec` / `delay_msec`, non-negative milliseconds of ROS time, so they last sim-seconds in simulation |

Missions refer to objects through references (`{buoy}`), never by repeating
coordinates. Today a reference is a map point from `StaticObject`; see
"Moving to a map" below for what changes once the boat has one.

Interrupting a maneuver (a `RosTimeout` firing, a failed step, Ctrl-C) always
releases guidance and zeroes `cmd_vel`.

Old data is refused rather than trusted. A cluster frame older than
`max_cluster_age` counts as seeing nothing, and a maneuver node will neither
lock on nor keep driving when the last odometry arrived more than
`max_odometry_age` ago (both 1 s, in `maneuvers.yaml`): it stops the boat and
fails.

## Moving to a map

Today every object reference is a stand-in: a map point
(`StaticObject point="x;y"`), resolved by locking onto the nearest lidar
cluster. When the boat has a map with stable object IDs, the switch touches
exactly:

- `ObjectRef` (`include/prop_mission_planner/object_ref.hpp`) gains an
  optional id, string form `"id:7"`;
- `ManeuverNode::try_lock` (`src/maneuver_node.cpp`) dispatches on it;
- `prop_maneuvers::TargetLock` gains `acquire_by_id` plus a subscription to
  the map, and `release()` must also clear ID-lock state;
- missions replace `StaticObject` lines with `FindObject`.

Nothing else in a mission changes. The map must give each object an ID when
it is first added, and never reuse or renumber it, in the `map` frame.

## Known limits

- The circle clearance check assumes the ideal path: the driven ring bulges
  inward, about 1.4 m measured on a 6 m, 4-leg ring.
- `BuoyTour` has not yet been driven end to end in simulation.
- While cluster frames are stale, obstacle avoidance sees nothing too, so a
  detour the approach has not yet committed to is dropped and the boat heads
  straight for its goal. The maneuver's own stale() check ends it, but only
  `reading_max_age` (5 s) after the last good refresh.
- The lidar must be stamped by the same clock as the mission planner: sim
  time in simulation (`bmp ... --sim`), and on the boat the host clock or a
  clock PTP-synced to it. Otherwise every frame looks old (or from the
  future), and the boat logs "cluster frame is N s old" and sees nothing.
