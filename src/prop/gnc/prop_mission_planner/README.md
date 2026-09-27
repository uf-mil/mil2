# prop_mission_planner

Behavior-tree missions for the boat. Same idea as the sub's `mission_planner`:
each mission is an XML tree, picked by name, run by one node that exits with
SUCCESS or FAILURE.

## Running a mission

    bmp BuoyTour --sim        # in simulation (use_sim_time)
    bmp BuoyTour              # on the boat

`bmp` (in `scripts/setup.bash`) passes `prop_maneuvers/config/maneuvers.yaml`
as the parameter file, so the maneuvers use the same tuning as the standalone
programs. Tab completion lists the installed missions.

Watch it live in Groot2 (port 1667, `-p groot_port:=0` to turn it off). If the
port is already busy, the node logs a warning and the mission runs without
Groot2. Every transition is also printed to the console.

Ctrl-C stops the boat and exits; a second Ctrl-C kills the process if
shutdown hangs.

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
coordinates. Today a reference is a map point from `StaticObject`; when the
boat has a map with object IDs, `StaticObject` lines become `FindObject` and
nothing else in the mission changes.

Interrupting a maneuver (a `RosTimeout` firing, a failed step, Ctrl-C) always
releases guidance and zeroes `cmd_vel`.
