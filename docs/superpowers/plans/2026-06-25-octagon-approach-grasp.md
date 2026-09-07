# S4 Approach & Grasp — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers-extended-cc:subagent-driven-development (recommended) or superpowers-extended-cc:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make SubjuGator approach, grasp, and lift a Task 5 object in sim, consuming `target_label` from S3 and emitting `grabbed_label` for the S6 loop.

**Architecture:** A pure-XML BehaviorTree.CPP v4 subtree (`ApproachAndGrasp`) composes the existing `HoneOverTarget`, `RelativeMove`, and the already-ported `ActuateServo` leaf. The sim gripper is made functional by extending the `GripperControl` gz-sim plugin to (a) provide the `Servo "gripper"` service that `ActuateServo` calls and (b) attach the nearest graspable prop on close / detach on open, with the four Task 5 props made dynamic.

**Tech Stack:** C++17, BehaviorTree.CPP v4, ROS 2 Jazzy (`rclcpp`), Gazebo Harmonic (gz-sim8), SDF worlds, colcon/ament.

**Design spec:** `docs/superpowers/specs/2026-06-25-octagon-approach-grasp-design.md`

**Prerequisite already done:** `ActuateServo` leaf cherry-picked onto `gripper-task-5` (commit `c0038125`) — generic `target`/`angle` servo-service leaf with clients in `Context`. This plan does **not** re-create it.

**Standing facts for the implementer:**
- Build with `source scripts/setup.bash && colcon build --packages-select <pkg>` (sets `GZ_VERSION=harmonic`; without it the gazebo build fails on gz-sim7).
- Cameras/rendering only work in a GUI Gazebo session, not agent-headless. The autonomous center→grasp loop additionally needs a Task 5 down-cam YOLO model that does **not exist yet** — so functional verification in this plan is limited to **build**, **mission instantiation**, and **manual (non-vision) grasp plumbing** in a GUI sim.
- Commits carry **no** AI self-attribution.

---

### Task 1: `ApproachAndGrasp` subtree (XML)

**Goal:** A reusable grasp subtree that centers over `target_label`, descends, closes, lifts, and records `grabbed_label`.

**Files:**
- Create: `src/subjugator/mission_planner/subjugator_missions/xml/approach_and_grasp.xml`
- Modify: `src/subjugator/mission_planner/subjugator_missions/xml/sub9_missions.xml`

**Acceptance Criteria:**
- [ ] `approach_and_grasp.xml` defines a `BehaviorTree ID="ApproachAndGrasp"` exactly as below.
- [ ] `sub9_missions.xml` includes it.
- [ ] `mission_planner` builds clean.

**Verify:** `source scripts/setup.bash && colcon build --packages-select mission_planner` → `Finished <<< mission_planner`. (Full parse is exercised by Task 2's instantiation.)

**Steps:**

- [ ] **Step 1: Create the subtree file**

`src/subjugator/mission_planner/subjugator_missions/xml/approach_and_grasp.xml`:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="ApproachAndGrasp">
    <!-- Fail-fast guard: never grab without a decided target (S3 sets target_label) -->
    <Precondition if="target_label != ''" else="FAILURE">
      <Sequence>
        <RetryUntilSuccessful num_attempts="{grasp_attempts}">
          <Sequence name="grasp_cycle">
            <!-- 1. Center over the selected object (graceful: degrades to dead-reckon) -->
            <SubTree ID="HoneOverTarget" _autoremap="true"
                     label="{target_label}" camera="down"
                     tol_norm="{center_tol_norm}" timeout_msec="{center_timeout_msec}"
                     ctx="{ctx}"/>
            <!-- 2. Ensure the gripper is open before descending -->
            <Action ID="ActuateServo" target="gripper" angle="{open_angle}" ctx="{ctx}"/>
            <!-- 3. Dead-reckon descent to grasp depth (negative z = down) -->
            <SubTree ID="RelativeMove" _autoremap="true"
                     z="{descend_z}" pos_tol="0.2" ori_tol_deg="180.0"
                     msec="{move_timeout_msec}" ctx="{ctx}"/>
            <!-- 4. Close to grasp, then let the joint/attach settle -->
            <Action ID="ActuateServo" target="gripper" angle="{close_angle}" ctx="{ctx}"/>
            <Delay delay_msec="{grip_settle_msec}"><AlwaysSuccess/></Delay>
            <!-- 5. Lift clear of the table (positive z = up) -->
            <SubTree ID="RelativeMove" _autoremap="true"
                     z="{lift_z}" pos_tol="0.2" ori_tol_deg="180.0"
                     msec="{move_timeout_msec}" ctx="{ctx}"/>
          </Sequence>
        </RetryUntilSuccessful>
        <!-- 6. Record what we grabbed -> S3's exclude reads this for the S6 loop -->
        <SetBlackboard output_key="grabbed_label" value="{target_label}"/>
      </Sequence>
    </Precondition>
  </BehaviorTree>
</root>
```

The knobs (`grasp_attempts`, `descend_z`, `lift_z`, angles, timeouts) are read from this subtree's blackboard and **supplied by the caller** — the Task 2 harness and the eventual production mission pass them as literals on the `<SubTree>` call (same pattern as `octagon_table_mission`). `target_label`/`grabbed_label`/`ctx` cross in via `_autoremap`.

- [ ] **Step 2: Register the include**

In `sub9_missions.xml`, add alongside the other `<include>` lines (e.g. right after the `hone_over_target.xml` include):

```xml
  <include path="approach_and_grasp.xml"/>
```

- [ ] **Step 3: Build**

Run: `source scripts/setup.bash && colcon build --packages-select mission_planner`
Expected: `Finished <<< mission_planner [..s]`, no CMake/parse error.

- [ ] **Step 4: Commit**

```bash
git add src/subjugator/mission_planner/subjugator_missions/xml/approach_and_grasp.xml \
        src/subjugator/mission_planner/subjugator_missions/xml/sub9_missions.xml
git commit -m "Add ApproachAndGrasp subtree for Task 5 S4"
```

---

### Task 2: `OctagonGraspMission` harness (XML) + instantiation smoke test

**Goal:** A standalone mission that seeds `target_label` and runs `ApproachAndGrasp`, used to smoke-test instantiation and (later, in a GUI) the full flow.

**Files:**
- Create: `src/subjugator/mission_planner/subjugator_missions/xml/octagon_grasp_mission.xml`
- Modify: `src/subjugator/mission_planner/subjugator_missions/xml/sub9_missions.xml`

**Acceptance Criteria:**
- [ ] `octagon_grasp_mission.xml` defines `BehaviorTree ID="OctagonGraspMission"` (a `main_tree_to_execute`) seeding `target_label` then calling `ApproachAndGrasp`.
- [ ] Included in `sub9_missions.xml`.
- [ ] Node instantiates the mission and reaches "Ticking tree" with no parse/registration error (this fully parses `ApproachAndGrasp` and exercises `ActuateServo` registration).

**Verify:** launch the node (below), publish one `/odometry/filtered`, observe `Mission Planner started. Ticking tree…` in the node log with no `Unknown mission` FATAL.

**Steps:**

- [ ] **Step 1: Create the mission file**

`src/subjugator/mission_planner/subjugator_missions/xml/octagon_grasp_mission.xml`:

```xml
<root BTCPP_format="4" main_tree_to_execute="OctagonGraspMission">
  <BehaviorTree ID="OctagonGraspMission">
    <Sequence>
      <!-- No upstream S3 in this harness; seed the target. TODO: real YOLO class. -->
      <SetBlackboard output_key="target_label" value="Pink_Bin"/>
      <!-- Knobs supplied at the call site (like octagon_table_mission); tune here.
           descend_z / lift_z / angles are sim-measured placeholders (spec §5.2). -->
      <SubTree ID="ApproachAndGrasp" _autoremap="true" ctx="{ctx}"
               grasp_attempts="2"
               center_tol_norm="0.05" center_timeout_msec="20000"
               open_angle="85" close_angle="0"
               descend_z="-0.6" lift_z="0.6"
               grip_settle_msec="1500" move_timeout_msec="30000"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- [ ] **Step 2: Register the include**

In `sub9_missions.xml`, add:

```xml
  <include path="octagon_grasp_mission.xml"/>
```

- [ ] **Step 3: Build**

Run: `source scripts/setup.bash && colcon build --packages-select mission_planner`
Expected: `Finished <<< mission_planner`.

- [ ] **Step 4: Instantiation smoke test**

Terminal A:
```bash
source scripts/setup.bash && source install/setup.bash
ros2 run mission_planner mission_planner_node --ros-args -p mission:=OctagonGraspMission
```
Expected: logs `Waiting for odometry...`.

Terminal B (publish a single odometry sample so the node proceeds to build/tick the tree):
```bash
source scripts/setup.bash && source install/setup.bash
ros2 topic pub --once /odometry/filtered nav_msgs/msg/Odometry \
  '{header: {frame_id: "odom"}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}}}}'
```
Expected in Terminal A: `Odometry received. Starting mission!` then `Mission Planner started. Ticking tree…`.
Failure signal to watch for: `FATAL ... Unknown mission 'OctagonGraspMission'` (parse/registration error) — must NOT appear.

Note: the node writes a debug `models.xml` to a hardcoded `/home/carlos/...` path; that write silently no-ops on other machines and is unrelated to success here.

- [ ] **Step 5: Commit**

```bash
git add src/subjugator/mission_planner/subjugator_missions/xml/octagon_grasp_mission.xml \
        src/subjugator/mission_planner/subjugator_missions/xml/sub9_missions.xml
git commit -m "Add OctagonGraspMission harness for Task 5 S4"
```

---

### Task 3: Make the four Task 5 props dynamic (world)

**Goal:** `Pink_Bin`, `Yellow_Bin`, `Yellow_Cup`, `Pink_Spoon` become non-static with inertia so the gripper can lift them; table/octagon stay static.

**Files:**
- Modify: `src/subjugator/simulation/subjugator_gazebo/worlds/robosub_2025.world`

**Acceptance Criteria:**
- [ ] Each of the four prop models has `<static>false</static>` and an `<inertial>` block on its `<link>`.
- [ ] `Task5_Table` and `Octagon_2025` remain `<static>true</static>`.
- [ ] `subjugator_gazebo` builds (installs the world).

**Verify:** `source scripts/setup.bash && colcon build --packages-select subjugator_gazebo` → `Finished`. GUI check (when a session is available): the four props rest on the table and can be nudged, while the table/octagon are immovable.

**Steps:**

- [ ] **Step 1: Edit each prop model**

For **each** of `Pink_Bin`, `Yellow_Bin`, `Yellow_Cup`, `Pink_Spoon` in `robosub_2025.world`: change `<static>true</static>` to `<static>false</static>`, and add an `<inertial>` block as the first child of that model's `<link name='link'>`. Example for `Pink_Bin` (apply the same inertial to all four):

```xml
<model name='Pink_Bin'>
  <static>false</static>
  <link name='link'>
    <inertial>
      <mass>0.3</mass>
      <inertia>
        <ixx>0.0005</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>0.0005</iyy><iyz>0</iyz><izz>0.0005</izz>
      </inertia>
    </inertial>
    <visual name='visual'>
      <!-- unchanged -->
    </visual>
    <collision name='collision'>
      <!-- unchanged -->
    </collision>
  </link>
  <pose>-6.403 14.58 -1 0 0 0</pose>
</model>
```

Leave `Task5_Table` and `Octagon_2025` exactly as they are (`<static>true</static>`, no inertial).

Note: `mass`/`inertia` are placeholders to tune in-GUI (TODO in spec §5.5) — if a prop sinks through or jitters, raise mass and/or inertia.

- [ ] **Step 2: Build**

Run: `source scripts/setup.bash && colcon build --packages-select subjugator_gazebo`
Expected: `Finished <<< subjugator_gazebo`.

- [ ] **Step 3: Commit**

```bash
git add src/subjugator/simulation/subjugator_gazebo/worlds/robosub_2025.world
git commit -m "Make Task 5 props dynamic for grasping"
```

---

### Task 4: `GripperControl` advertises the `Servo "gripper"` service

**Goal:** The sim gripper responds to the same `/gripper` `Servo` service `ActuateServo` calls (real `driver.py` provides it on hardware), mapping `angle` → joint open/closed.

**Files:**
- Modify: `src/subjugator/simulation/subjugator_gazebo/include/GripperControl.hh`
- Modify: `src/subjugator/simulation/subjugator_gazebo/src/GripperControl.cc`
- Modify: `src/subjugator/simulation/subjugator_gazebo/CMakeLists.txt`

**Acceptance Criteria:**
- [ ] Plugin advertises a `subjugator_msgs/srv/Servo` service named `gripper` on its ROS node.
- [ ] `angle=0` drives joints to `closed_pos_`; `angle=85` drives them to `open_pos_`; values in between interpolate.
- [ ] `subjugator_gazebo` builds.

**Verify:** `colcon build --packages-select subjugator_gazebo`. GUI: with sim running, `ros2 service call /gripper subjugator_msgs/srv/Servo "{angle: 85}"` opens the gripper; `"{angle: 0}"` closes it (visible joint motion).

**Steps:**

- [ ] **Step 1: Header — include, constant, member, method**

In `GripperControl.hh`, add to the includes (near the `std_msgs` include):

```cpp
#include <subjugator_msgs/srv/servo.hpp>
#include <algorithm>
```

Add the service member next to `key_sub_`:

```cpp
    rclcpp::Service<subjugator_msgs::srv::Servo>::SharedPtr gripper_srv_;
```

Add a constant near `open_pos_`/`closed_pos_`:

```cpp
    static constexpr double OPEN_ANGLE{ 85.0 };  // angle that maps to fully open
```

Declare the callback next to `KeypressCallback`:

```cpp
    void GripperCallback(std::shared_ptr<subjugator_msgs::srv::Servo::Request> const req,
                         std::shared_ptr<subjugator_msgs::srv::Servo::Response> res);
```

- [ ] **Step 2: Advertise the service in `Configure`**

In `GripperControl.cc`, immediately after `this->key_sub_ = ...create_subscription...` in `Configure`:

```cpp
    this->gripper_srv_ = this->node_->create_service<subjugator_msgs::srv::Servo>(
        "gripper", std::bind(&GripperControl::GripperCallback, this, std::placeholders::_1,
                             std::placeholders::_2));
    std::cout << "[GripperControl] Advertised Servo service '/gripper'" << std::endl;
```

- [ ] **Step 3: Implement the callback**

Add to `GripperControl.cc` (e.g. after `KeypressCallback`):

```cpp
void GripperControl::GripperCallback(std::shared_ptr<subjugator_msgs::srv::Servo::Request> const req,
                                     std::shared_ptr<subjugator_msgs::srv::Servo::Response> /*res*/)
{
    double frac = static_cast<double>(req->angle) / OPEN_ANGLE;
    frac = std::clamp(frac, 0.0, 1.0);
    double const tgt = this->closed_pos_ + frac * (this->open_pos_ - this->closed_pos_);

    // Service callback runs inside spin_some() in PostUpdate (gz thread), same
    // thread as PreUpdate — safe to set targets directly.
    this->left_target_pos_ = tgt;
    this->right_target_pos_ = tgt;
    this->gripper_open_ = frac > 0.5;

    std::cout << "[GripperControl] Servo cmd angle=" << static_cast<int>(req->angle) << " -> target=" << tgt
              << std::endl;
}
```

- [ ] **Step 4: CMake — add the dependency**

In `subjugator_gazebo/CMakeLists.txt`, change the GripperControl dependency line:

```cmake
ament_target_dependencies(GripperControl rclcpp std_msgs subjugator_msgs)
```

(`subjugator_msgs` is already `find_package`d and in `package.xml`.)

- [ ] **Step 5: Build**

Run: `source scripts/setup.bash && colcon build --packages-select subjugator_gazebo`
Expected: `Finished <<< subjugator_gazebo`.

- [ ] **Step 6: Commit**

```bash
git add src/subjugator/simulation/subjugator_gazebo/include/GripperControl.hh \
        src/subjugator/simulation/subjugator_gazebo/src/GripperControl.cc \
        src/subjugator/simulation/subjugator_gazebo/CMakeLists.txt
git commit -m "Add Servo gripper service to GripperControl sim plugin"
```

---

### Task 5: Proximity attach-on-close / detach-on-open in `GripperControl`

**Goal:** When the gripper closes, the nearest allowlisted prop within `attach_radius` of the gripper link is held and lifts with the sub; when it opens, the prop is released.

**Files:**
- Modify: `src/subjugator/simulation/subjugator_gazebo/include/GripperControl.hh`
- Modify: `src/subjugator/simulation/subjugator_gazebo/src/GripperControl.cc`
- Modify: `src/subjugator/simulation/subjugator_description/urdf/sub9_sim.urdf.xacro` (plugin SDF params)

**Acceptance Criteria:**
- [ ] Plugin reads an SDF allowlist of graspable model names and an `attach_radius`.
- [ ] On a close command, the nearest allowlisted model whose origin is within `attach_radius` of the `gripper_link` world pose is "held"; on an open command it is released.
- [ ] A held prop tracks the gripper and lifts with the sub in a GUI session; releasing drops it.
- [ ] Table/octagon are never held (not in the allowlist).

**Verify:** GUI sim — drive the sub so the gripper is over `Pink_Bin`, `ros2 service call /gripper ... "{angle: 0}"` → bin follows the gripper as the sub rises (use a teleop or `ros2 topic pub /goal_pose` to lift); `"{angle: 85}"` → bin drops.

> **Implementation note (spec §5.4 risk):** This task delivers the **kinematic-follow** hold mechanism — robust and fully specified — as the initial implementation, satisfying the spec's sanctioned fallback. The optional physics fixed-joint upgrade is described at the end; it swaps only the hold/release internals behind the same allowlist + proximity logic and should be attempted in-GUI where its gz API can be validated. Holding the prop kinematically while it is grasped is acceptable because the prop is only constrained during the carry.

**Steps:**

- [ ] **Step 1: Header — grasp state + helpers**

In `GripperControl.hh` add includes:

```cpp
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Pose3.hh>
#include <vector>
#include <unordered_set>
```

Add members in the private section:

```cpp
    // Proximity grasp
    std::unordered_set<std::string> graspable_names_;
    double attach_radius_{ 0.25 };
    std::string gripper_link_name_{ "gripper_link" };
    gz::sim::Entity gripper_link_entity_{ gz::sim::kNullEntity };
    gz::sim::Entity held_model_entity_{ gz::sim::kNullEntity };
    gz::math::Pose3d held_offset_;  // gripper_link -> held model, captured at grasp
    bool want_grasp_{ false };      // last commanded state: true=closed/grasp
    bool grasp_active_{ false };    // currently holding something

    void TryGrasp(gz::sim::EntityComponentManager &ecm);
    void ReleaseGrasp();
```

- [ ] **Step 2: Parse SDF allowlist + radius in `Configure`**

In `GripperControl.cc` `Configure`, inside the `if (sdf)` block (after the joint-name parsing):

```cpp
        if (sdf->HasElement("attach_radius"))
        {
            this->attach_radius_ = sdf->Get<double>("attach_radius");
        }
        if (sdf->HasElement("gripper_link_name"))
        {
            this->gripper_link_name_ = sdf->Get<std::string>("gripper_link_name");
        }
        for (auto el = sdf->FindElement("graspable"); el; el = el->GetNextElement("graspable"))
        {
            this->graspable_names_.insert(el->Get<std::string>());
        }
        std::cout << "[GripperControl] " << this->graspable_names_.size()
                  << " graspable models, attach_radius=" << this->attach_radius_ << std::endl;
```

Also cache the gripper link entity in the existing `ecm.Each<components::Name>` lambda by adding a branch:

```cpp
            if (_name && _name->Data() == this->gripper_link_name_)
            {
                this->gripper_link_entity_ = _ent;
            }
```

- [ ] **Step 3: Drive grasp/release from the command, hold in `PreUpdate`**

In `GripperCallback` (Task 4), set the intent flag at the end:

```cpp
    this->want_grasp_ = frac <= 0.5;  // closing => grasp intent
```

In `PreUpdate`, after the existing joint-smoothing block, add:

```cpp
    // Edge-triggered grasp / release
    if (this->want_grasp_ && !this->grasp_active_)
    {
        this->TryGrasp(_ecm);
    }
    else if (!this->want_grasp_ && this->grasp_active_)
    {
        this->ReleaseGrasp();
    }

    // While holding, keep the prop locked to the gripper (kinematic follow)
    if (this->grasp_active_ && this->held_model_entity_ != gz::sim::kNullEntity &&
        this->gripper_link_entity_ != gz::sim::kNullEntity)
    {
        gz::sim::Link link(this->gripper_link_entity_);
        auto linkPose = link.WorldPose(_ecm);
        if (linkPose)
        {
            gz::math::Pose3d const target = *linkPose * this->held_offset_;
            gz::sim::Model(this->held_model_entity_).SetWorldPoseCmd(_ecm, target);
        }
    }
```

- [ ] **Step 4: Implement `TryGrasp` / `ReleaseGrasp`**

```cpp
void GripperControl::TryGrasp(gz::sim::EntityComponentManager &ecm)
{
    if (this->gripper_link_entity_ == gz::sim::kNullEntity)
        return;
    gz::sim::Link gripperLink(this->gripper_link_entity_);
    auto gpOpt = gripperLink.WorldPose(ecm);
    if (!gpOpt)
        return;
    gz::math::Pose3d const gripperPose = *gpOpt;

    // Find nearest allowlisted model within attach_radius
    gz::sim::Entity best = gz::sim::kNullEntity;
    double bestDist = this->attach_radius_;
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](gz::sim::Entity const &ent, gz::sim::components::Model const *,
            gz::sim::components::Name const *name) -> bool
        {
            if (!name || this->graspable_names_.find(name->Data()) == this->graspable_names_.end())
                return true;
            auto mpOpt = gz::sim::Model(ent).WorldPose(ecm);
            if (!mpOpt)
                return true;
            double const d = (mpOpt->Pos() - gripperPose.Pos()).Length();
            if (d < bestDist)
            {
                bestDist = d;
                best = ent;
            }
            return true;
        });

    if (best == gz::sim::kNullEntity)
        return;

    auto mpOpt = gz::sim::Model(best).WorldPose(ecm);
    this->held_model_entity_ = best;
    this->held_offset_ = gripperPose.Inverse() * (*mpOpt);  // gripper_link -> model
    this->grasp_active_ = true;
    std::cout << "[GripperControl] Grasped model entity " << best << " at dist " << bestDist << std::endl;
}

void GripperControl::ReleaseGrasp()
{
    this->grasp_active_ = false;
    this->held_model_entity_ = gz::sim::kNullEntity;
    std::cout << "[GripperControl] Released grasp" << std::endl;
}
```

Add the needed component include at the top of the `.cc` if not present:

```cpp
#include <gz/sim/components/Model.hh>
```

- [ ] **Step 5: Pass SDF params to the plugin**

In `sub9_sim.urdf.xacro`, extend the existing `GripperControl` plugin block:

```xml
<plugin filename="GripperControl" name="gripper_control::GripperControl">
  <left_joint_name>gripper_leftArm_joint</left_joint_name>
  <right_joint_name>gripper_rightArm_joint</right_joint_name>
  <gripper_link_name>gripper_link</gripper_link_name>
  <attach_radius>0.25</attach_radius>
  <graspable>Pink_Bin</graspable>
  <graspable>Yellow_Bin</graspable>
  <graspable>Yellow_Cup</graspable>
  <graspable>Pink_Spoon</graspable>
</plugin>
```

- [ ] **Step 6: Build**

Run: `source scripts/setup.bash && colcon build --packages-select subjugator_gazebo subjugator_description`
Expected: `Finished` for both. Resolve any gz-sim API signature mismatches (`WorldPose`, `SetWorldPoseCmd`, `Each<...>`) against the installed gz-sim8 headers if the compiler flags them — the logic is unchanged, only the exact call shape may need a tweak.

- [ ] **Step 7: GUI functional check + commit**

In a GUI sim, perform the Verify steps above (over a prop → close → lift follows → open → drop). Then:

```bash
git add src/subjugator/simulation/subjugator_gazebo/include/GripperControl.hh \
        src/subjugator/simulation/subjugator_gazebo/src/GripperControl.cc \
        src/subjugator/simulation/subjugator_description/urdf/sub9_sim.urdf.xacro
git commit -m "Add proximity attach/detach to GripperControl sim plugin"
```

**Optional follow-up — physics fixed-joint upgrade:** replace the kinematic-follow block (Step 3's hold loop) and `TryGrasp`/`ReleaseGrasp` internals with a runtime detachable fixed joint: on grasp, create a `gz::sim::components::DetachableJoint` (or equivalent ECM joint entity) linking `gripper_link` to the held model's link; on release, remove it. The allowlist + proximity selection are unchanged. Validate in-GUI; if joint creation is unreliable, keep kinematic-follow.

---

## Notes on verification scope

- Tasks 1–2 are fully verifiable headless (build + instantiation smoke test).
- Tasks 3–5's functional behavior requires a **GUI** Gazebo session (per the rendering/physics constraint); their **build** steps are headless-verifiable.
- The **autonomous** S4 loop (`HoneOverTarget` → descend → grasp end-to-end) is gated on a Task 5 down-cam YOLO model that does not exist yet; it is out of scope for this plan and tracked in the spec (§8).
