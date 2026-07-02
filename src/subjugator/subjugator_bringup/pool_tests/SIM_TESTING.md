# Task 5 sim testing guide (down-cam centering, target select, gripper, grasp)

This walks a brand-new teammate, on a brand-new machine, from nothing to running
the Task 5 down-cam stack in Gazebo: centering over the table (S2), selecting a
target (S3), the gripper plugin, and the approach-and-grasp mission (S4).

You do not need to know ROS, the sub, or the mission planner beforehand. Follow
the parts in order. Each part ends with "you should see ..." — if you don't, jump
to **Troubleshooting** at the bottom before continuing.

---

## READ THIS FIRST — everything runs on `gripper-task-5`

The whole vision + grasp test runs from a **single branch: `gripper-task-5`.** It
contains all three pieces you need:

| Piece you need | Where it is |
|---|---|
| Mission stack: `pooltest.sh`, hone / select / combined / grasp missions, the gripper plugin with the `/gripper` Servo service | `gripper-task-5` |
| The trained down-cam YOLO model `octagon_sim.pt` (detects the table + the objects) | `gripper-task-5` |
| A Gazebo world the down-cam can detect (`table_2026/`: table + `nut_cylinder`, `pill_cylinder`, `electric_box`, `bandaid_box`) | `gripper-task-5` |

(Historically these lived on separate branches — `jack-task5-downcam-yolo` had the
model and world, `gripper-task-5` had the missions. They have since been merged,
so `gripper-task-5` is the one branch that runs everything. If you're on an older
checkout that's missing a piece, just `git fetch` and check out the latest
`gripper-task-5`.)

Every vision stage — even the simplest centering test — needs the YOLO model,
because the sub centers on the **`table`** detection. The world props, the YOLO
classes, and the gripper's graspable allowlist (`grasp_targets.yaml`) all use the
**same real names** now (`nut_cylinder`, `electric_box`, `pill_cylinder`,
`bandaid_box`), so select → grasp lines up end to end.

```bash
export TASK5_BRANCH=gripper-task-5
```

> **What works without a display.** The gripper hardware seam (Part 7) and the
> grasp motion (Part 8) exercise the `/gripper` service and proximity attach;
> those don't need YOLO. But every vision stage (Parts 4–6 and the full
> end-to-end run) needs the down-cam to render frames, which needs a graphical
> session (see Part 0).

---

## Part 0 — Machine setup (skip if ROS 2 Jazzy + Gazebo Harmonic already work)

You need **Ubuntu 24.04** (Noble). The stack targets:

- **ROS 2 Jazzy** at `/opt/ros/jazzy`
- **Gazebo Harmonic** (gz-sim 8)
- A **graphical session** (real monitor, or a desktop over RDP/x11 — **not** a
  bare SSH terminal). The down-cam only renders frames when there's a display;
  headless, you'll get an empty camera and zero detections.

Install ROS 2 Jazzy (desktop) following the official docs:
<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>

Install Gazebo Harmonic + the ROS↔Gazebo bridge:

```bash
sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz colcon
```

Quick check — both should print versions without error:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help >/dev/null && echo "ros2 ok"
gz sim --version
```

---

## Part 1 — Get the code and the test branch

```bash
# Pick a workspace home; this guide uses ~/mil2
git clone <the mil2 repo URL> ~/mil2
cd ~/mil2

# Check out the branch that has everything
git checkout "$TASK5_BRANCH"            # gripper-task-5
git submodule update --init --recursive   # the repo uses submodules
```

**Confirm the branch really has all three pieces** before building (5 seconds,
saves you an hour):

```bash
# 1) the YOLO model
ls src/subjugator/gnc/subjugator_vision/models/octagon_sim.pt

# 2) the detectable world objects
ls src/subjugator/simulation/subjugator_description/models/table_2026/

# 3) the mission stack
ls src/subjugator/subjugator_bringup/pool_tests/pooltest.sh
```

If any of those three is missing, you're on a stale checkout — `git fetch origin`
and re-checkout the latest `gripper-task-5`.

---

## Part 2 — Build the workspace

The repo ships an environment script. **You must source it before building** — it
sets `GZ_VERSION=harmonic`, sources ROS, and points Gazebo at the right version.
Building without it fails looking for the wrong gz-sim.

```bash
cd ~/mil2
source scripts/setup.bash      # sets GZ_VERSION=harmonic, sources /opt/ros/jazzy, etc.
colcon build                   # first build is slow (10–20 min); grab coffee
source install/setup.bash      # make the freshly built packages visible
```

You should end with `Summary: N packages finished` and no `Failed` lines.

> Re-sourcing rule for **every new terminal** you open later:
> ```bash
> cd ~/mil2 && source scripts/setup.bash && source install/setup.bash
> ```

---

## Part 3 — Launch the simulator

This opens the Gazebo GUI, spawns Sub9, loads the Task 5 world, and starts the
ROS↔Gazebo bridge.

**Terminal 1:**

```bash
cd ~/mil2 && source scripts/setup.bash && source install/setup.bash
ros2 launch subjugator_bringup gazebo.launch.py
```

You should see: the Gazebo window open with the pool, the octagon, the table, and
Sub9 in the water. Leave it running.

**Sanity check — the camera streams are alive (Terminal 2):**

```bash
cd ~/mil2 && source scripts/setup.bash && source install/setup.bash
ros2 topic hz /down_cam/image_raw      # down camera (what S2/S3 uses)
ros2 topic hz /front_cam/image_raw     # front camera
```

Both should report a steady rate (e.g. ~10–30 Hz). If `/down_cam/image_raw` is
silent, your session has no display — see Troubleshooting.

---

## Part 4 — Launch the down-cam YOLO node

The missions read detections from **`/yolo_down/detections`**. The stock sim YOLO
launch points at the *front* camera with the *start-gate* model under namespace
`yolo`, so you must override three things: the model, the input image, and the
namespace.

**Terminal 3:**

```bash
cd ~/mil2 && source scripts/setup.bash && source install/setup.bash

ros2 launch yolo_bringup yolov11_sim.launch.py \
  model:=$(ros2 pkg prefix subjugator_vision)/share/subjugator_vision/models/octagon_sim.pt \
  input_image_topic:=/down_cam/image_raw \
  namespace:=yolo_down \
  device:=cpu
```

`device:=cpu` is fine for testing; pass `device:=0` only if you have a CUDA GPU.

**Sanity check — detections are flowing and have the right class names (Terminal 4):**

```bash
cd ~/mil2 && source scripts/setup.bash && source install/setup.bash
ros2 topic hz /yolo_down/detections                       # should publish steadily
ros2 topic echo --once /yolo_down/detections | grep class_name
```

With the sub positioned over the table you should see class names from this set:

```
table  nut_cylinder  electric_box  pill_cylinder  bandaid_box  red_cross  warning
```

If `/yolo_down/detections` publishes but is always empty, the sub isn't looking at
the table yet (that's fine — the test stages position it), or the model/world
don't match (see Troubleshooting).

---

## Part 5 — Preflight (no motion)

From here on, the guided runner `pooltest.sh` drives the missions. It was built
for poolside testing but has a `--sim` flag that wires the missions to the sim
camera topic. Run **preflight first** — it just confirms the rig, moves nothing.

**Terminal 5:**

```bash
cd ~/mil2 && source scripts/setup.bash && source install/setup.bash
cd src/subjugator/subjugator_bringup/pool_tests
./pooltest.sh preflight --sim
```

You should see `down image`, `detections`, and `odometry` all reporting a rate,
the currently-detected classes listed, and `PREFLIGHT OK`. If anything says
`MISSING`, fix it (Parts 3–4) before moving on.

> `./pooltest.sh --help` lists every stage and flag. `--dry-run` prints what a
> stage would do without launching anything — handy to read the script's plan.

---

## Part 6 — Down-cam stages: center, then select (S2 + S3)

Run these one at a time, in order. Each is a self-contained mission. Position the
sub first (next subsection), then run the stage. Results are written under
`pooltest_runs/<stage>_<UTC>/` (bag, console log, a results form).

### Positioning the sub in sim

There's no diver in sim, so you place the sub with the Gazebo GUI:

- In the Gazebo window, use the **Translate** tool (or the Entity Tree → Sub9 →
  Pose fields) to move Sub9 so the **table is centered under the down camera,
  ~1 m above it**.
- Verify in Terminal 4 that `/yolo_down/detections` shows `table` while the sub
  hovers there.

### 6a — `calib`: camera sign / gain calibration

```bash
./pooltest.sh calib --sim
# type GO when the sub is positioned over the table
```

Success: the sub drives to reduce the table's offset from image center and holds.
If it drives **away** from the table, the axis signs are flipped — the stage
prints the file and the `swap_axes` / `map_x_sign` / `map_y_sign` knobs to flip,
then rebuild `mission_planner` and re-run. Record the sign triple that converged.

### 6b — `hone`: hold center over the table

```bash
./pooltest.sh hone --sim
# GO
```

Success: the sub centers over the table and holds using the calibrated signs. If
it loses the table it should give up gracefully (falls back, doesn't thrash).

### 6c — `select`: lock the target object (needs a role)

A **role** picks which objects this run grabs:

| role | objects it will consider |
|---|---|
| `survey_repair` | `nut_cylinder`, `electric_box` |
| `search_rescue` | `pill_cylinder`, `bandaid_box` |

Position the sub so the **objects** are in the down-cam frame, then:

```bash
./pooltest.sh select --sim --role survey_repair
# GO
```

Success: the console prints `SelectTarget: locked '<class>' (N frames)`. Record
the locked label and whether it's the right object.

### 6d — `combined`: center then select in one run (the S2+S3 milestone)

```bash
./pooltest.sh combined --sim --role survey_repair
# GO
```

Success: it centers over the table, **then** prints `locked '<class>'`. This is
the headline S2+S3 result.

---

## Part 7 — Test the gripper directly (no YOLO needed)

The Gazebo `GripperControl` plugin advertises a **`Servo`** service at
**`/gripper`**. The `angle` field is the shared hardware/sim unit (PWM duty ×10):
the plugin maps it to the gripper joint.

With the sim running (Part 3), in any sourced terminal:

```bash
# confirm the service exists
ros2 service list | grep /gripper

# open / close the gripper — watch the claw move in the Gazebo window
ros2 service call /gripper subjugator_msgs/srv/Servo "{angle: 82}"    # idle
ros2 service call /gripper subjugator_msgs/srv/Servo "{angle: 200}"   # one direction
ros2 service call /gripper subjugator_msgs/srv/Servo "{angle: 1000}"  # other extreme
```

Each call should print `[GripperControl] Servo cmd angle=... -> target=...` in
Terminal 1 and visibly move the claw. (`angle` range is 0–1000.)

**Proximity attach/detach:** drive the gripper so a graspable prop sits between
the claws, then send the close angle — the plugin should attach the prop to the
gripper (it follows the claw). Open it and the prop should detach and fall. The
allowlist of grab-able props is `mission_planner/config/grasp_targets.yaml`
(`nut_cylinder`, `electric_box`, `pill_cylinder`, `bandaid_box` — the same names
as the world props and the YOLO classes).

---

## Part 8 — Approach-and-grasp mission (S4)

The grasp mission `OctagonGraspMission` (subtree `ApproachAndGrasp`) centers over
the chosen target, opens the gripper, descends, and closes. It will **refuse to
grab without a decided target** (`target_label` must be set — normally by the
S3 select stage).

```bash
# from the pool_tests dir, with sim + YOLO running
ros2 run mission_planner mission_planner_node --ros-args \
  -p mission:=OctagonGraspMission \
  -p role:=survey_repair \
  -p down_image_topic:=/down_cam/image_raw
```

Success: the sub selects a target, centers, opens, descends, closes, and the prop
is attached to the gripper.

> **Names line up end to end.** `grasp_targets.yaml` (the graspable allowlist),
> the `<model name>` props in `robosub_2025.world`, and the `octagon_sim.pt` YOLO
> classes all use the **same real names** (`nut_cylinder`, `electric_box`,
> `pill_cylinder`, `bandaid_box`). So the select → grasp handoff matches: the
> object YOLO locks is a graspable prop in the world. (This used to be a known
> mismatch when the world carried placeholder props like `Pink_Bin`; that's been
> reconciled.) The world props are `static=false` with box collision so the
> gripper can actually pick them up.

---

## Part 9 — Full S2 dead-reckon mission (optional, Tier B)

`OctagonTableMission` runs the whole S2 approach: surface → descend →
dead-reckon to the table → hone. It needs four dead-reckon constants baked into
`octagon_table_mission.xml`. In sim those are start-pose specific. Measure them
with the `deadreckon` stage (drive to each checkpoint, type `SNAP`), edit the
printed constants into the mission XML, rebuild `mission_planner`, then:

```bash
./pooltest.sh full_s2 --sim
# GO
```

Success: the sub ends centered over the table. This is heavier and least likely
to "just work" first try; treat it as a stretch goal after Parts 6–8.

---

## Part 10 — Place (S5), the collection loop (S6), and the full mission (S1→S7)

These build directly on Parts 6–8.

**S5 — transport & place** (`OctagonPlaceMission`): pretends the sub is already
holding an object, moves to the role's basket, centers on the basket marker, and
opens the gripper to drop it. Needs the basket markers detectable (`warning` for
`survey_repair`, `red_cross` for `search_rescue`).

```bash
ros2 run mission_planner mission_planner_node --ros-args \
  -p mission:=OctagonPlaceMission -p role:=survey_repair \
  -p down_image_topic:=/down_cam/image_raw
```

**S6 — collection loop** (`OctagonLoopMission`): the real thing — S3 select → S4
grasp → S5 place, repeated for up to two objects. The second pass automatically
skips the object already placed. Position the sub centered over the table with the
objects **and** the basket marker in the down-cam frame:

```bash
./pooltest.sh loop --sim --role survey_repair
# GO   (sub grabs, carries, drops — twice; one object still counts)
```

**Full mission** (`OctagonMission`): the whole task, S1→S7, in one tree. It
**defaults to running everything** — no flags needed. In sim there is usually no
pinger, so skip S1 with `--no-pinger` and hand-place the sub over the octagon; dial
how far it goes with `--score-level`:

```bash
# hand-placed, run up to "grab & place 2 objects" (no S7):
./pooltest.sh full --sim --no-pinger --score-level 3 --role survey_repair
# GO
```

`score_level` ladder: `0` pinger→table · `1` +center · `2` +grab 1 · `3` +grab 2 ·
`5` +face image (S7) · `6` +rotation bonus (default). S7 needs an `octagon_symbols`
forward-cam model; without it the run still places the objects and then fails
honestly at S7's model check (end-of-run log says `FAILURE`, but collection scored).

The node prints `Running mission '<name>' (role=…, score_level=…, do_pinger=…)` at
startup — check it to confirm your params actually applied. (Bare `-p name:=value`
is silently treated as a remap; `pooltest.sh` and the commands above use the correct
`--ros-args -p` form.)

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `/down_cam/image_raw` silent, `ros2 topic hz` hangs | No graphical session. The camera needs a display — run on a real desktop or an x11/RDP desktop, not bare SSH. |
| `/yolo_down/detections` publishes but is always empty | Sub isn't over the table yet (reposition), **or** the world has the plain table instead of `table_2026` (stale checkout — re-pull `gripper-task-5`), **or** wrong model path in Part 4. |
| YOLO node errors on the model path | The `model:=` path doesn't exist. Re-run the `ls .../octagon_sim.pt` check from Part 1; rebuild so it's installed to the share dir. |
| `Unknown mission '<name>'` | You added/changed missions but didn't rebuild. `colcon build --packages-select mission_planner && source install/setup.bash`. |
| `colcon build` fails looking for `gz-sim7` / wrong Gazebo | You didn't `source scripts/setup.bash` first (it sets `GZ_VERSION=harmonic`). Source it, then rebuild. |
| Sub sits still during a motion stage | No detections (`table` not visible), or odometry not publishing. Re-run `./pooltest.sh preflight --sim`. |
| Stage says it ran but nothing moved | Check the stage's `console.log` under `pooltest_runs/<stage>_<UTC>/` for the real error. |
| Grasp closes but the prop doesn't attach | The prop name must be in `grasp_targets.yaml` **and** match the world `<model name>`. Both should be the real classes (`nut_cylinder`, …); a stale checkout may still have placeholder props. |
| `/gripper` service not found | The `GripperControl` plugin isn't loaded — you're on a branch/world without it, or the sub didn't spawn. Confirm Part 3 launched cleanly. |

---

## Appendix A — quick reference

**Topics**

| topic | what |
|---|---|
| `/down_cam/image_raw` | down camera image (sim) |
| `/front_cam/image_raw` | front camera image (sim) |
| `/yolo_down/detections` | down-cam YOLO detections (S2–S6 read this) |
| `/yolo/detections` | forward-cam YOLO detections (S7 wall image) |
| `/odometry/filtered` | fused pose |

**Service**

| service | type | what |
|---|---|---|
| `/gripper` | `subjugator_msgs/srv/Servo` | move the gripper; `angle` 0–1000 (idle ≈ 82) |

**Missions (`-p mission:=`)**

| name | does |
|---|---|
| `CenterCameraTest` | calib — sign/gain check |
| `HoneOverTableOnly` | hone — hold center over the table |
| `SelectOnly` | select — lock the role's target |
| `HoneOverTableSelect` | combined — center then select (S2+S3) |
| `OctagonGraspMission` | approach + grasp (S4) |
| `OctagonPlaceMission` | transport + place into the basket (S5) |
| `OctagonLoopMission` | collection loop — grab + place up to 2 objects (S6) |
| `OctagonTableMission` | full S2 surface→descend→dead-reckon→hone |
| `OctagonMission` | **the whole task, S1→S7** (dials: `score_level` 0–6 default 6, `do_pinger` 1/0) |

**Roles**

| role | objects |
|---|---|
| `survey_repair` | `nut_cylinder`, `electric_box` |
| `search_rescue` | `pill_cylinder`, `bandaid_box` |

**Grasp target / world prop / YOLO class names** (one shared set):
`nut_cylinder  electric_box  pill_cylinder  bandaid_box`

**pooltest.sh stages** (append `--sim`, and `--role <r>` where a target is needed):
`preflight  calib  hone  select  combined  deadreckon  full_s2  loop  full`

`full` extras: `--score-level N` (0–6, default 6) and `--no-pinger` (skip S1).
