# prop_maneuvers

Reusable boat maneuvers, meant to be called by a mission planner:

- **face an object** — hold position and turn until the front points at it
- **circle an object** — go all the way round using straight legs (a diamond)
- **approach an object** — face it, drive to it, stop short, step around
  anything in the way

Each is a library function plus a standalone program you can run and watch.

## How it drives

The boat drives like a tank: two motors at the back, no sideways movement.
Maneuvers run strictly one at a time so only one thing commands the motors:
the `Driver`, which hands waypoints, and optionally a final heading, to
`prop_controller`'s `guidance` on `plan`. `guidance` does the driving,
turning (a one-waypoint plan at the boat) and reversing.

Backing away is a `Driver` goal straight behind the boat. `guidance` aims the
stern at a final waypoint inside its `reverse_cone` of dead astern and commands
negative speed, so the boat keeps its heading and the target stays in front of
the lidar. Outside the cone it turns and drives forward as usual.

A maneuver that finishes or fails holds station (a one-waypoint plan at the
boat), so guidance keeps publishing zero speed and thrust is not cut.

A reverse moves the boat, and `guidance` has no sensors, so `clear_behind()` is
checked before the reverse starts and again on every step. The check runs on
live data rather than a snapshot taken before setting off: the boat can see
straight out the back, through the gap between the pontoons, so there is
current information to check against.

## How it finds the buoy

There is no tracker and no persistent ID numbers, because the boat always knows
where it is. One sighting is converted into map coordinates and remembered; the
boat's own position estimate carries it while the buoy is out of sight. To
refresh, the boat turns to face the remembered point — which puts the buoy
straight ahead, clear of the blind spots — and takes the blob nearest to where
it expected one.

Finding the buoy for the first time is more limited than refreshing it.
Acquisition looks at what the lidar can see at that moment — inside the
forward cone, within range — and gives up if nothing is there. It never turns
to search, because turning to face something requires having already found
it, which is exactly what the refresh step above depends on doing first.
Pointing the boat roughly the right way before a maneuver starts is the
caller's job, not this package's.

## How it steps around an obstacle

Approach plans past an obstacle with a straddle: two waypoints, one before it
and one after, rather than a single waypoint. A single waypoint only reaches its full
sideways offset as the boat draws level with the obstacle, so the path bulges
inward on the way there. Two waypoints finish the sideways move early and
hold it past, so the clearance asked for is the clearance actually driven. Their spacing equals
the sideways offset itself, the tightest spacing that still met the number
across 1,959 swept obstacle positions.

Clearance is measured hull-to-buoy-surface, not centre-to-centre, and split
into two numbers:

- `min_gap` (0.5 m) decides only WHETHER to step around something at all.
- `detour_clearance` (0.5 m) is how far out the detour actually swings, once
  one is needed.

Keep them equal, or nearly so: if the swing is larger than the trigger, the
boat reacts only once it is already inside the room it needs. 0.5 m is a
placeholder for testing. The narrowest gate in the RobotX handbook is 1.83 m
between buoys, so large clearances cannot be delivered there.

Two distances no amount of waypoint planning can change: the start and the
goal, because every path begins where the boat actually is and ends where it
was told to go. If either already sits inside the clearance, no detour
recovers it. The start can be fixed by reversing away first; the goal cannot
be, so the boat takes the best route available and logs what it actually
achieved instead of refusing to move.

## Settings

Everything tunable is in `config/maneuvers.yaml`. The hull and sensor block is
at the top: **change the blind spot angles there if the lidar moves.** Where
the lidar is physically bolted is not in that file — it lives in
`prop_localization/urdf/prop.urdf` and reaches this code through the position
chain at runtime.

`hull_half_width` (0.443 m), `hull_behind` (0.367 m) and `hull_front`
(0.760 m) were measured on the boat on 2026-09-08. `hull_behind` is to the
propellers, and the pontoons extend further back, so it is optimistic until
the pontoon tails are measured.
