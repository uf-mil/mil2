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

- **driving** is delegated to `prop_controller`'s `guidance` by publishing a
  point list on `plan`, and switched off again by publishing an **empty** list
- **turning in place** is done by publishing `cmd_vel` directly, which
  `guidance` cannot do

Releasing the driver is not optional. If a maneuver skips it, `guidance` keeps
sending "hold still" commands ten times a second and they interleave with the
turn commands, making the boat stutter.

## How it finds the buoy

There is no tracker and no persistent ID numbers, because the boat always knows
where it is. One sighting is converted into map coordinates and remembered; the
boat's own position estimate carries it while the buoy is out of sight. To
refresh, the boat turns to face the remembered point — which puts the buoy
straight ahead, clear of the blind spots — and takes the blob nearest to where
it expected one.

## Settings

Everything tunable is in `config/maneuvers.yaml`. The hull and sensor block is
at the top: **change the blind spot angles there if the lidar moves.** Where
the lidar is physically bolted is not in that file — it lives in
`prop_localization/urdf/prop.urdf` and reaches this code through the position
chain at runtime.
