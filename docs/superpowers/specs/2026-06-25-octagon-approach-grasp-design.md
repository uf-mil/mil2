# S4 — Approach & Grasp (RoboSub Task 5, Restore/Octagon)

**Date:** 2026-06-25
**Branch:** `gripper-task-5`
**Status:** Design approved; ready for implementation plan.

## 1. Context & goal

Task 5 (Restore/Octagon) has SubjuGator surface inside an octagon, retrieve
objects from a table, place them in baskets, and orient toward images. The
mission is decomposed into stages S0–S7:

- S1 pinger approach → **S2** surface & center over the table → **S3**
  detect/select object → **S4 approach & grasp** → S5 transport & place →
  S6 second-object loop → S7 face image + yaw.

S2 and S3 are already built on this branch. **S4 is this design.** Its job:
consume the `target_label` chosen by S3, approach and grasp that object, lift it
clear of the table, and hand a held object to S5 — and to be **functionally
runnable in the GUI sim end-to-end** (not just structurally complete).

The mission planner is BehaviorTree.CPP v4: C++ leaf nodes in
`mission_planner/subjugator_operations/{include,src}`, reusable subtrees and
missions as XML in `mission_planner/subjugator_missions/xml/`, registered via
`<include>` in `sub9_missions.xml` and `registerNodeType` in
`mission_planner/src/mission_planner_node.cpp`.

## 2. Scope

**In scope (S4):**
- Center over the selected object (reuse `HoneOverTarget` with
  `label={target_label}`).
- Dead-reckon descent to grasp depth.
- Gripper actuation (close to grasp, open to release) via the existing
  `ActuateServo` leaf.
- Lift clear of the table (terminal action of S4).
- Record `grabbed_label` for the S6 two-object loop.
- Make the grasp *functional in sim*: a `Servo` command path into the sim
  gripper, an attach mechanism so a closed gripper actually holds an object,
  and dynamic (pick-uppable) objects.

**Out of scope (deferred):**
- S3 selection logic (already built; S4 only consumes `target_label`).
- S5 transport & placement, and the release-into-basket (S5 opens the gripper).
- A Task 5 down-cam YOLO model (does not exist on any branch; see §8).
- Sim command paths for the dropper/torpedo `Servo` services (Task 3/4 work).

## 3. Key decisions

| # | Decision | Rationale |
|---|---|---|
| Scope | **Full functional grasp in sim** (not BT-layer-only). | User chose the most complete option; makes the command seam and attach physics real design problems. |
| Command seam | **Preserve the real chain**: BT → `Servo "gripper"` service → provider. Sim provider = `GripperControl` plugin; real provider = `driver.py`. | Byte-identical command path real vs sim; only the service provider differs. The C++ stack calls the `Servo` service **directly** — the `mechanism` action + `mechanisms_server.py` is the legacy Python path and is **not** used. |
| Actuation leaf | **Reuse `ActuateServo`** (generic over `target` = dropper/gripper/torpedo + `angle`). No new actuation leaf. | The leaf already exists (Ethan mechanisms, #500) on `task5-work`; cherry-picked onto this branch. Serves Tasks 3/4 too. |
| Grasp physics | **Proximity fixed-joint attach** (primary), **kinematic-follow** (fallback). | Physically real grasp; runtime joint creation is the one fiddly piece, so keep a robust escape hatch with the same external interface. |
| Descent | **Dead-reckon to grasp depth** via `RelativeMove`. | Down-cam loses the object when close; objects sit on the table at a known z; the attach radius absorbs small error. Matches existing `RelativeMove` placeholder pattern. |
| Confirm + handoff | **Close → lift to clear → open-loop success**, set `grabbed_label = target_label`. | The `Servo` call has no grasp feedback (matches real hardware). The lift is part of S4; S5 owns transport/place. |
| Failure stance | **Fail-fast with bounded retry**: require `target_label` (FAIL if empty), retry the cycle up to N times, return FAILURE if still unable. Object-centering still degrades gracefully internally. | Grabbing without a decision, or transporting nothing, is worse than stopping (consistent with S3). |
| Graspable set | **All four props dynamic**; plugin attaches the single nearest allowlisted model within the attach radius on close. | General, supports the S6 two-object loop, label-agnostic (keeps the command seam action/service-only; the plugin needs no mission state). |

## 4. Architecture

Two layers joined only by the `/gripper` `Servo` service:

```
BT layer (mission_planner, C++/XML)            Sim layer (subjugator_gazebo + world)
─────────────────────────────────             ────────────────────────────────────
ApproachAndGrasp (subtree XML)
  Precondition: target_label != ''
  RetryUntilSuccessful(grasp_attempts):
    Sequence:
      HoneOverTarget(label={target_label})  ── down-cam YOLO (existing, gated on model)
      ActuateServo(gripper, open) ──────────┐
      RelativeMove(z=descend)               │
      ActuateServo(gripper, close) ─────────┤  /gripper (Servo srv)
      Delay(settle)                         │        ↓
      RelativeMove(z=lift)                  └─> GripperControl plugin (sim):
  SetBlackboard grabbed_label := target_label    - advertises Servo "gripper"
                                                  - angle → joint open/closed
                                                  - attach nearest allowlisted prop
                                                    within attach_radius on close;
                                                    detach on open
                                                 World: 4 props now dynamic
```

`target_label` (blackboard, from S3) feeds `HoneOverTarget`'s `label` and, on
success, is copied to `grabbed_label` (blackboard), which S3's `exclude` reads
to skip the already-grabbed object on the S6 loop. The attach is
**label-agnostic** (nearest allowlisted prop), so the plugin never needs mission
state.

## 5. Components

### 5.1 `ActuateServo` leaf — DONE (cherry-picked)

`BT::StatefulActionNode`, ports `target` (dropper|gripper|torpedo), `angle`
(int → uint16), `ctx`. Calls the `Servo` service for the chosen target via
clients held in `Context`; RUNNING until the response arrives, then SUCCESS;
FAILURE on missing ctx/target, unknown target, or service-not-ready.

Ported byte-identical from `task5-work` (so the branches reconcile cleanly when
they converge), plus minimal infra: three `Servo` clients in `Context`, their
creation + `registerNodeType<ActuateServo>` in `mission_planner_node.cpp`, the
`subjugator_msgs` dependency in `CMakeLists.txt`/`package.xml`, and
`Servo.srv` bumped `uint8`→`uint16` to match upstream. Builds clean
(`colcon build --packages-select subjugator_msgs mission_planner`).

### 5.2 `ApproachAndGrasp` subtree — NEW (XML, no C++)

`subjugator_missions/xml/approach_and_grasp.xml`. Composes `HoneOverTarget`,
`RelativeMove`, and `ActuateServo` with BTCPP-v4 built-ins (`Precondition`,
`RetryUntilSuccessful`, `SetBlackboard`, `Delay`). Cycle ordering:
center → open → descend → close → settle → lift; then record `grabbed_label`.

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="ApproachAndGrasp">
    <Precondition if="target_label != ''" else="FAILURE">
      <Sequence>
        <RetryUntilSuccessful num_attempts="{grasp_attempts}">
          <Sequence name="grasp_cycle">
            <SubTree ID="HoneOverTarget" _autoremap="true"
                     label="{target_label}" camera="down"
                     tol_norm="{center_tol_norm}" timeout_msec="{center_timeout_msec}"
                     ctx="{ctx}"/>
            <Action ID="ActuateServo" target="gripper" angle="{open_angle}" ctx="{ctx}"/>
            <SubTree ID="RelativeMove" _autoremap="true"
                     z="{descend_z}" pos_tol="0.2" ori_tol_deg="180.0"
                     msec="{move_timeout_msec}" ctx="{ctx}"/>
            <Action ID="ActuateServo" target="gripper" angle="{close_angle}" ctx="{ctx}"/>
            <Delay delay_msec="{grip_settle_msec}"><AlwaysSuccess/></Delay>
            <SubTree ID="RelativeMove" _autoremap="true"
                     z="{lift_z}" pos_tol="0.2" ori_tol_deg="180.0"
                     msec="{move_timeout_msec}" ctx="{ctx}"/>
          </Sequence>
        </RetryUntilSuccessful>
        <SetBlackboard output_key="grabbed_label" value="{target_label}"/>
      </Sequence>
    </Precondition>
  </BehaviorTree>
</root>
```

Tunable knobs (placeholders, measured in GUI sim like `octagon_table_mission`'s
distances):

| Knob | Placeholder | Meaning |
|---|---|---|
| `grasp_attempts` | `2` | bounded retries of the cycle |
| `center_tol_norm` / `center_timeout_msec` | `0.05` / `20000` | object-centering tolerance / timeout |
| `descend_z` | `-0.6` **TODO** | descent from hover to grasp range |
| `lift_z` | `0.6` **TODO** | lift to clear the table |
| `open_angle` / `close_angle` | `85` / `0` | gripper open / closed (mapping in §5.4) |
| `grip_settle_msec` | `1500` | let the close + attach take effect |
| `move_timeout_msec` | `30000` | per-`RelativeMove` timeout |

**Notes:**
- Fail-fast is **open-loop**: the cycle FAILs (→ retry → eventual FAILURE) on a
  move/actuation failure (`RelativeMove` timeout, gripper service not ready) or
  empty `target_label` — not on "object not actually held," since the grab is
  not verified (matches the open-loop confirmation decision).
- `Precondition` uses BTCPP scripting (built into v4) and assumes `target_label`
  *exists* on the blackboard (seeded by S3 or the test mission).
- Retry compounds depth: attempt 2 re-centers and descends again from wherever
  attempt 1 ended. Acceptable for move/service failures; documented limitation,
  no bookkeeping added now.

### 5.3 `OctagonGraspMission` harness — NEW (XML)

`subjugator_missions/xml/octagon_grasp_mission.xml`. Standalone S4 tuning
harness (as `CenterCameraTest` is for S2): no upstream S3, so it seeds
`target_label` and runs `ApproachAndGrasp`.

```xml
<root BTCPP_format="4" main_tree_to_execute="OctagonGraspMission">
  <BehaviorTree ID="OctagonGraspMission">
    <Sequence>
      <SetBlackboard output_key="target_label" value="Pink_Bin"/>  <!-- TODO: real YOLO class -->
      <SubTree ID="ApproachAndGrasp" _autoremap="true" ctx="{ctx}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Both new XML files registered via `<include>` in `sub9_missions.xml`. The
production full-Task-5 mission (S2→S3→S4 chained) is a separate, later file;
`ApproachAndGrasp` is built to drop into it.

### 5.4 `GripperControl` plugin — EXTENDED (`subjugator_gazebo`)

Currently keyboard-only (`/keyboard/keypress` `'u'` toggle), position-only, no
attach. Three additions:

1. **Advertise the `Servo "gripper"` service** on the plugin's existing
   `rclcpp::Node` (resolves to `/gripper`, matching `ActuateServo`'s client and
   real `driver.py`). The callback runs inside `spin_some` in `PostUpdate`, i.e.
   on the gz thread — same thread as `PreUpdate`, so it sets member targets
   directly with no cross-thread races. Mapping:
   `target = closed_pos_ + clamp(angle/OPEN_ANGLE, 0, 1) * (open_pos_ − closed_pos_)`,
   `OPEN_ANGLE = 85` ⇒ `angle=0` fully closed (grasp), `angle=85` fully open.
   Keyboard `'u'` stays as a manual fallback.

2. **Proximity attach-on-close / detach-on-open.** On a close command (target
   crosses below a grip threshold), find the nearest **allowlisted** model
   within `attach_radius` of `gripper_link`'s world pose and create a fixed
   joint (driving the physics `AttachFixedJoint` primitive via the ECM — the
   same primitive gz's `DetachableJoint` system uses); break it on open. The
   allowlist is SDF-configured (`<graspable>Pink_Bin</graspable>` ×4) so the
   plugin can never attach the sub, table, or octagon.
   **Fallback (same external interface):** if runtime joint creation is
   unreliable in gz Harmonic, swap the attach/detach internals for
   *kinematic-follow* (write the held model's pose to track `gripper_link` each
   `PreUpdate`). Implement fixed-joint first; keep kinematic-follow as the
   escape hatch. Only the ~30 lines that "hold the object" differ; the service,
   BT, subtree, and props are identical either way.

   SDF knobs: `attach_radius` (≈`0.25 m` **TODO**), grip threshold.

### 5.5 World — props go dynamic (`robosub_2025.world`)

`Pink_Bin`, `Yellow_Bin`, `Yellow_Cup`, `Pink_Spoon`: `<static>true</static>` →
`false`, add `<inertial>` (mass ≈`0.3 kg` **TODO** + simple box inertia, tuned so
they rest on the table and lift cleanly). `Task5_Table` and `Octagon_2025` stay
static (and are excluded from the attach allowlist).

## 6. Data flow (end to end)

```
ApproachAndGrasp → ActuateServo(target=gripper, angle=0)
   → /gripper (Servo srv) → GripperControl callback
      → joints close  +  attach nearest allowlisted prop within radius
RelativeMove(lift_z) → sub rises → fixed joint carries the prop up
SetBlackboard grabbed_label := target_label
[S5 later] ActuateServo(angle=85) → joints open + detach → prop released
```

## 7. Error handling

- **No `target_label`:** `Precondition` returns FAILURE immediately (no blind
  grab).
- **Centering can't see the object:** `HoneOverTarget` degrades to dead-reckon
  (graceful, internal) — the cycle continues to descend/grasp.
- **Move/actuation failure** (`RelativeMove` timeout, gripper service not
  ready): the cycle FAILs → `RetryUntilSuccessful` re-attempts up to
  `grasp_attempts` → FAILURE if still unable. S4 returns FAILURE (not masked).
- **Grab not actually achieved:** not detected (open-loop). Accepted tradeoff;
  a vision/again-detect confirmation is a future improvement.

## 8. Dependencies, assumptions, risks

- **No Task 5 down-cam YOLO model exists** on any branch (committed models are
  front-cam Task 1/2 only). The grasp *plumbing* (§5.4, §5.5 + `ActuateServo`)
  is buildable and testable now via manual driving; the **autonomous**
  `HoneOverTarget → descend → grasp` loop is gated on that model, exactly like
  S2/S3.
- **Branch divergence:** `gripper-task-5` (S2/S3) and `task5-work` (mechanisms)
  have diverged; S4 needs pieces from both. Resolved by a **surgical
  cherry-pick** of `ActuateServo` onto this branch (done), not a full merge —
  `task5-work` carries no S5/S6/S7 logic and no Task 5 vision, so a full merge
  would add unrelated Task 2/4/start-gate/CI churn for one leaf.
- **Runtime fixed-joint creation** is the highest-risk implementation step;
  mitigated by the kinematic-follow fallback (§5.4).
- **Cameras render only in a GUI Gazebo session**, not agent-headless — the
  autonomous functional run requires a GUI session.

## 9. Verification

- **Build:** `colcon build --packages-select subjugator_msgs mission_planner`
  and the `subjugator_gazebo` plugin.
- **Instantiate:** `ros2 run mission_planner mission_planner_node --ros-args -p
  mission:=OctagonGraspMission`, publish one `/odometry/filtered`, confirm it
  reaches "Ticking tree" with no parse/registration error (now exercising
  `ActuateServo`).
- **Plumbing (no YOLO), GUI sim:** drive the sub over a prop manually (teleop or
  scripted `RelativeMove`), call `/gripper angle:=0` → prop attaches and lifts
  with the sub; `angle:=85` → prop releases.
- **Autonomous (gated on a Task 5 model):** the full center→descend→grasp loop.

## 10. Conventions

- Leaf nodes named by mechanism (`ActuateServo`), subtrees by intent
  (`ApproachAndGrasp`); mission/subtree XML named by behavior, no `S4` shorthand
  in shared files.
- Commits carry **no** AI self-attribution.
