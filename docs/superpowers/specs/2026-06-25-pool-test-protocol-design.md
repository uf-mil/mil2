# Pool Test Protocol (S2 + S3) — Design

**Date:** 2026-06-25
**Branch:** gripper-task-5
**Status:** Approved design, ready for implementation plan

## Goal

A single guided shell script the team runs poolside to test the Task 5 down-cam
stack modularly — smallest whole unit first, each stage a superset of the last —
while capturing the pool-specific information we can only get in the water
(sign/axis calibration, gains, detection confidence/range, settle behavior, and
optionally the dead-reckon constants). The required milestone is an end-to-end
**S2 + S3** run: center over the table, then select the role target.

## Scope

- **Tier A (required):** preflight → calibrate → hone → select → combined.
  The combined run (center → lock target in one mission process) is the S2 + S3
  milestone.
- **Tier B (optional):** measure the four dead-reckon constants, then run full S2
  (`OctagonTableMission`) with them. Gated on having the task objects/models;
  not required for the S2 + S3 milestone.

## Architecture

One guided shell script, `pooltest.sh`, run on the sub's onboard machine. It
walks an ordered ladder of stages. For each stage it:

1. Prints the **precondition** (how the sub must be placed) and waits for the
   operator to type `GO` (so a diver/teleop can position the sub first).
2. Launches the stage's **mission** (real-robot topic defaults — no sim
   overrides).
3. Records a **bag** of the relevant topics and **tees** the mission console to
   a log.
4. Prints a **results form** the operator fills in (the eyeball observations a
   machine can't easily judge from a bag).

The script supports running a single stage (`./pooltest.sh calib`) or the guided
sequence (`./pooltest.sh`).

### Capture mechanisms

- **Bag** (`ros2 bag record`): flight recorder of the data topics
  (`/odometry/filtered`, `/goal_pose`, `/yolo_down/detections`) so convergence,
  timing, and detection confidence can be measured offline. Raw images excluded
  by default (large).
- **tee**: splits the mission's stdout to screen *and* `console.log`, preserving
  the node's own narration (e.g. `SelectTarget: locked '<x>' (N frames)`).
- **Results form**: a short fill-in template printed at the end of each stage for
  human observations (converged? oscillated? signs flipped?), appended to
  `results.txt`.

## Stage ladder

### Tier A — required (S2 + S3 milestone)

| # | Stage | Mission | New? | Proves / extracts |
|---|-------|---------|------|-------------------|
| 0 | Preflight | *(checks only, no motion)* | — | down image + `/yolo_down/detections` flowing; odom healthy; real topic names + image WxH; detection rate; classes the down model produces |
| 1 | Calibrate | `CenterCameraTest` | exists | converging `swap_axes`/`map_x_sign`/`map_y_sign`; tuned `kp`/`max_step`; achievable `tol_norm`; settle behavior. Smallest whole unit. |
| 2 | Hone | `HoneOverTableOnly` | new | degrade-wrapped subtree holds center over `table`; table detection confidence + altitude/range |
| 3 | Select | `SelectOnly` | new | role → `{target_label}` lock on real detections; good `min_conf`/`consecutive_frames`; per-class confidence; false picks |
| 4 | **Combined** | `HoneOverTableSelect` | new (fused) | center → lock in one process, exercising the in-tree `{target_label}` handoff. **S2 + S3 milestone.** |

### Tier B — optional (full S2 with real constants)

| # | Stage | Mission | New? | Extracts |
|---|-------|---------|------|----------|
| 5 | Measure dead-reckon | *(guided odom-snapshot procedure)* | — | `surface_dz`, `hover_dz`, `octagon_dx`, `octagon_dy` |
| 6 | Full S2 | `OctagonTableMission` | exists | full surface → descend → dead-reckon → hone with measured constants |

### Boundaries

- Stages 1–3 are isolated; the sub is repositioned between them. Stage 4 chains
  internally with no reposition.
- Stage 4 is the deliverable milestone. Tier B is explicitly optional.
- **Dependency:** Stages 3–4 need trained YOLO models for the role's object
  classes (`survey_repair` → nut_bolt/plug; `search_rescue` → pill/bandage) and
  the `role` param set; Stage 2 needs a `table` model. A missing model degrades
  that stage to "topics connect, no detection" — the script detects and reports
  this rather than silently passing.

## Per-stage capture spec

Common: launch uses real-robot topic defaults (no `down_image_topic` /
`down_detect_topic` overrides — those are sim-only). Each stage records a bag of
`/odometry/filtered`, `/goal_pose`, `/yolo_down/detections` and tees the node
console to `console.log`. Output folder: `pooltest_runs/<stage>_<UTC-stamp>/`.

- **Stage 0 — Preflight** (no motion, gates only). Checks `/yolo_down/detections`
  publishing + rate, down image topic publishing + actual WxH, `/odometry/filtered`
  healthy, `yolo_down` node alive; prints classes the down model currently
  produces. **Pass:** all topics live, detection rate ≥ ~5 Hz, expected classes
  present. Fails loud if any missing.

- **Stage 1 — Calibrate** · `mission:=CenterCameraTest`. Precondition: sub
  hovering ~1 m over table, roughly level. Signs/gains are literals in the XML;
  operator edits them between attempts. Form: converged toward target (y/n)? ·
  signs used · `kp`/`max_step` used · final tightness (tight/oscillated/drifted) ·
  approx settle time · notes. **Pass:** error shrinks within `tol_norm` and holds.
  Headline output: the converging sign triple + gains.

- **Stage 2 — Hone** · `mission:=HoneOverTableOnly`. Precondition: same as Stage 1.
  Form: held center (y/n)? · table detection confidence range · approx altitude ·
  detection stable/flickery · did it fall back to AlwaysSuccess (lost table)?
  **Pass:** stays centered using Stage-1 signs via the real subtree wrapper.

- **Stage 3 — Select** · `mission:=SelectOnly` · `-p role:=<survey_repair|search_rescue>`.
  Precondition: sub positioned so the task objects are in the down-cam frame; role
  chosen (script prompts/arg). Form: locked label · correct? · lock time/frames ·
  per-class confidence · false picks · `min_conf`/`consecutive_frames` used.
  **Pass:** locks the correct role-class. Output: tuned `min_conf`/`consecutive_frames`.

- **Stage 4 — Combined** · `mission:=HoneOverTableSelect` · `-p role:=…`.
  Precondition: sub hovering over table with objects present. Form: centered then
  locked (y/n)? · locked label · total time · handoff worked (`{target_label}`
  populated)? **Pass:** one process centers over the table then locks the target.

- **Stage 5 — Measure dead-reckon** (Tier B). See procedure below. Writes
  `constants.txt`.

- **Stage 6 — Full S2** · `mission:=OctagonTableMission` with measured constants
  edited in.

## New mission XML

Three thin wrappers around already-registered nodes/subtrees — **no new C++**.
Each gets an `<include>` in `sub9_missions.xml`.

`hone_over_table_only_mission.xml` (Stage 2):

```xml
<root BTCPP_format="4" main_tree_to_execute="HoneOverTableOnly">
  <BehaviorTree ID="HoneOverTableOnly">
    <SubTree ID="HoneOverTarget" label="table" camera="down"
             tol_norm="0.05" timeout_msec="30000" ctx="{ctx}"/>
  </BehaviorTree>
</root>
```

`select_only_mission.xml` (Stage 3 — wrapper binds the subtree's ports so it runs
as a root):

```xml
<root BTCPP_format="4" main_tree_to_execute="SelectOnly">
  <BehaviorTree ID="SelectOnly">
    <SubTree ID="SelectTargetS3" ctx="{ctx}" target_label="{target_label}"/>
  </BehaviorTree>
</root>
```

`hone_over_table_select_mission.xml` (Stage 4 — fused milestone; remaps
`{target_label}` to root scope so the handoff is real and observable):

```xml
<root BTCPP_format="4" main_tree_to_execute="HoneOverTableSelect">
  <BehaviorTree ID="HoneOverTableSelect">
    <Sequence>
      <SubTree ID="HoneOverTarget" label="table" camera="down"
               tol_norm="0.05" timeout_msec="30000" ctx="{ctx}"/>
      <SubTree ID="SelectTargetS3" ctx="{ctx}" target_label="{target_label}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

The locked class is already logged by `SelectTarget`, so it lands in
`console.log` — no extra logging node needed.

## Dead-reckon measurement procedure (Stage 5, Tier B)

The four constants are *relative* moves from wherever S1 leaves the sub, so the
procedure pins a start pose and snapshots odom at labeled checkpoints; the script
computes the deltas and writes `constants.txt`. The operator does the moving
(teleop/diver); the script snapshots one `/odometry/filtered` on command.

1. Place sub at the designated **S1-arrival start pose** (a pool marker). Snapshot
   → `A = (xA, yA, zA)`.
2. Surface the sub. Snapshot → `zS`. ⇒ `surface_dz = zS − zA`.
3. Descend to down-cam hover depth (table fills frame). Snapshot → `zH`. ⇒
   `hover_dz = zH − zS`.
4. Translate until centered over the table. Snapshot → `(xC, yC)`. ⇒
   `octagon_dx = xC − xA`, `octagon_dy = yC − yA`.

**Synergy:** after a Stage 4 run the sub is already centered over the table — its
end pose *is* checkpoint 4, so Stage 5 can offer to reuse it.

**Validity flag (written into `constants.txt`):** these values are valid only for
that start pose; re-measure at competition against the real S1 arrival.

## File layout

```
src/subjugator/mission_planner/subjugator_missions/xml/
    hone_over_table_only_mission.xml        (new — Stage 2)
    select_only_mission.xml                 (new — Stage 3)
    hone_over_table_select_mission.xml      (new — Stage 4 fused)
  + sub9_missions.xml                       (3 new <include> lines)

src/subjugator/subjugator_bringup/pool_tests/
    pooltest.sh                             (the guided runner)
    README.md                               (ladder + how to run + how to read outputs)
```

Output per run, in `./pooltest_runs/<stage>_<UTC-stamp>/` (gitignored, path
overridable via env var):

```
<stage>_<stamp>/
    bag/                 ros2 bag of odom, goal_pose, yolo_down detections
    console.log          teed mission stdout
    results.txt          the filled-in results form
    constants.txt        (Stage 5 only) the four measured dead-reckon values
```

## Script internals (bash)

- Stage registry: each stage = precondition text + launch command + bag topic
  list + form fields.
- Arg parsing: `./pooltest.sh [stage|all]`.
- `GO` gate before each stage.
- Interactive `role` prompt for Stages 3–4 (or `--role` arg).
- Bag start/stop wrapping each run; `tee` for console.
- `read`-driven form appended to `results.txt`.
- Odom-snapshot helper (`ros2 topic echo --once /odometry/filtered`) for Stage 5.
- Preflight via `ros2 topic list` / `ros2 topic hz`.

### `--sim` rehearsal flag

Same ladder, but adds the sim overrides (`down_image_topic:=/down_cam/image_raw`)
and reminds the operator to launch the `yolo_down` node. Lets the whole harness —
staging, GO gates, bagging, forms, and that all four missions instantiate — be
rehearsed in a GUI Gazebo session before pool day. `--sim` only rehearses the
harness; real runs use the real-robot topic defaults.

## Non-goals

- Does **not** manually position the sub (initial placement or repositioning
  between stages) — a diver/teleop does that. The script *does* launch missions
  that command autonomous motion (centering, dead-reckon), which is the behavior
  under test. Stage 5 is the exception: there all motion is manual teleop and the
  script only snapshots odom.
- Does **not** train or load YOLO models — assumes they are present; reports
  clearly if a class never appears.
- Does **not** auto-tune gains/signs — operator edits `CenterCameraTest` literals
  between calibration attempts (auto-parse deferred).
- Does **not** cover S1 pinger approach or S4 grasp.
- Not the sim test path itself — `--sim` only rehearses the harness.
- Does **not** bag raw images by default (large; Stage 0 may grab a few throttled
  frames if a visual record is wanted).
