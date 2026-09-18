# prop_maneuvers

Reusable boat maneuvers, meant to be called by a mission planner:

- **face an object** — hold position and turn until the front points at it
- **circle an object** — go all the way round using straight legs (a diamond)
- **approach an object** — face it, drive to it, stop short, step around
  anything in the way

Each is a library function plus a standalone program you can run and watch.

## How it drives

The boat drives like a tank: two motors at the back, no sideways movement.
Maneuvers run strictly one at a time so only one thing commands the motors,
through one of three primitives:

| | how it moves the boat | what it does |
|---|---|---|
| `Driver` | hands waypoints to `prop_controller`'s `guidance` on `plan` | drives forward along straight legs |
| `Spinner` | `cmd_vel` direct | turns on the spot, does not move |
| `Reverser` | `cmd_vel` direct | backs straight up, holding heading, does not turn |

`guidance` cannot back the boat up: its speed is `speed * max(0, cos(error))`,
so a negative forward speed never comes out the other end. Handed a point
behind the boat it pivots 180 degrees and drives forward instead, sweeping the
hull through the water being avoided and taking about four times as long.
`Reverser` exists to skip that turn and keep the target in front of the lidar
the whole time.

Releasing the driver is not optional. Both `Spinner` and `Reverser` need
`driver.release()` first — switching `Driver` off by publishing an **empty**
list on `plan` — or `guidance` keeps sending "hold still" commands ten times a
second and they interleave with the direct commands, making the boat stutter.

`Reverser` additionally needs `clear_behind()` checked before the reverse
starts and again on every step, because it has no sensors of its own and will
happily back into a buoy otherwise. Unlike `Spinner`, which only turns the
boat and cannot hit anything, `Reverser` actually moves it. The check runs on
live data rather than a snapshot taken before setting off — the boat can see
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
and one after, rather than the single waypoint used before. The boat drives
start → waypoint → goal, and a single waypoint only reaches its full sideways
offset at the moment the boat draws level with the obstacle — the path bulges
inward on the way there. Measured: asking for 1.25 m of clearance delivered
only 1.05 m. Two waypoints finish the sideways move early and hold it past, so
the clearance asked for is the clearance actually driven. Their spacing equals
the sideways offset itself, the tightest spacing that still met the number
across 1,959 swept obstacle positions.

Clearance is measured hull-to-buoy-surface, not centre-to-centre, and split
into two numbers that used to be one:

- `min_gap` (0.15 m) decides only WHETHER to step around something at all.
- `detour_clearance` (1.0 m) is how far out the detour actually swings, once
  one is needed.

The old number was 2.0 m from the boat's CENTRE, which was undeliverable: the
narrowest gate in the RobotX handbook is 1.83 m between buoys, and demanding
2 m of clearance from each one needs 4 m of room in a 1.83 m gap.

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

`hull_half_width` (0.5 m) and `hull_behind` (1.0 m) are ASSUMPTIONS, not
measurements — taken from the competition size box (a USV must fit within
2 x 1 x 1 m, handbook p88) with `base_link` assumed to sit in the middle of
the boat. Nothing establishes that it does. Measure the real boat and replace
both once it's available.
