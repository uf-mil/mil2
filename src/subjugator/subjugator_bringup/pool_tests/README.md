# Pool tests — Task 5 octagon mission (S1 → S7)

Guided poolside runner for the whole Task 5 stack: acoustic homing (S1), down-cam
centering (S2), target selection (S3), grasp (S4), transport & place (S5), the
two-object collection loop (S6), and orient-to-image + rotation bonus (S7). Start
with the smallest stages and work up to the full mission. Run it on the sub's
onboard machine. See the design specs under `docs/superpowers/specs/`
(`2026-06-25-pool-test-protocol-design.md`, `2026-07-01-octagon-full-mission-design.md`).

## Run it (always under tmux so it survives disconnect)

```bash
tmux new -s pooltest
source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
cd src/subjugator/subjugator_bringup/pool_tests
./pooltest.sh                 # guided sequence (smallest stage first)
./pooltest.sh calib           # one stage
./pooltest.sh loop            # S6: grab + place two objects
./pooltest.sh full            # the WHOLE Task 5 run, S1..S7 (default = everything)
./pooltest.sh full --no-pinger --score-level 3   # hand-placed, up to 2 objects, no S7
./pooltest.sh --help          # all stages + flags
```

Replace `<ws>` with your ROS 2 workspace root (e.g. `~/mil2`).

## The whole mission in one command

`./pooltest.sh full` launches `OctagonMission`, which by **default runs the entire
Task 5 sequence** — you do not need to remember any flag for the competition run.
Two optional dials help bring-up:

| `--score-level N` | runs every phase up to N, then stops |
|:---:|---|
| 0 | S1 pinger → table |
| 1 | + S2 center over table |
| 2 | + grab & place 1 object |
| 3 | + grab & place 2 objects (S6 loop) |
| 5 | + S7 face the octagon-wall image |
| **6 (default)** | + S7 rotation bonus (full run) |

`--no-pinger` skips S1 and starts at S2 — use it when there is no active pinger and
you hand-place the sub over the octagon. (With the default, S1 sweeps for a ping and
will not proceed until it hears one.)

The `full` run's table acquisition (S2) is **calibration-free**: it descends until
the down-cam sees the table and centers on it with vision, so — unlike the `full_s2`
stage — it needs **no** dead-reckon constants. You do not have to run `deadreckon`
before `full`.

**After you type `GO` you have 30 s (default) to disconnect and clear the sub.**
It then runs autonomously. Detach tmux with `Ctrl-b d`; reattach with
`tmux attach -t pooltest` to fill in the results form.

Rehearse the whole flow dry, with no hardware: add `--dry-run`. Rehearse in
Gazebo: add `--sim` (also launch a `yolo_down` node on `/down_cam/image_raw`).

## Roles

| role | object classes |
|------|----------------|
| `survey_repair` | nut_cylinder, electric_box |
| `search_rescue` | pill_cylinder, bandaid_box |

Pass with `--role survey_repair` (or you'll be prompted).

## Required YOLO model(s) per stage

| stage | needs model for |
|-------|-----------------|
| preflight | (just checks the stream is alive) |
| calib, hone | `table` |
| select | the role's object classes |
| combined | `table` + the role's object classes |
| deadreckon | none (manual teleop) |
| full_s2 | `table` |
| loop | `table` + role object classes + basket markers (`warning`/`red_cross`) |
| full | all of the above + `octagon_symbols` (forward cam) for S7 |

If the `octagon_symbols` model is missing, `full` still grabs and places the
objects (those score); S7 then fails honestly at its model check and the
end-of-mission log reads `FAILURE` even though collection succeeded.

## One-page checklist

| stage | POSITION the sub | TYPE | EXPECT | RECORD |
|-------|------------------|------|--------|--------|
| preflight | n/a | (none) | all topics live | MISSING items |
| calib | ~1 m over table, level | `GO` | self-centers & holds | converging signs + gains |
| hone | ~1 m over table | `GO` | holds center | confidence, altitude, stability |
| select | objects in down-cam view | role (flag or prompt), `GO` | `locked '<class>'` | locked label, conf, frames |
| combined | ~1 m over table, objects in view | role (flag or prompt), `GO` | centers then locks | label, total time, handoff y/n |
| deadreckon | drive to each CP in order | `SNAP` x4 | constants printed | constants.txt values |
| full_s2 | S1-arrival start pose | `GO` | ends over table | which step failed |
| loop | centered over table, objects + basket in view | role, `GO` | grabs & drops 1–2 objects | objects placed, labels |
| full | in water near octagon (or hand-placed + `--no-pinger`) | role, `GO` | runs S1..S7 | farthest stage, objects placed |

## Outputs

Each run writes `pooltest_runs/<stage>_<UTC>/` containing `bag/`, `console.log`,
`results.txt` (and `constants.txt` for deadreckon). This folder is gitignored.

## Glossary

- **hone** — autonomously drive the sub so the target sits at the image center
  (closed-loop visual centering on the down camera).
- **role** — which object set this mission grabs (`survey_repair` vs
  `search_rescue`); selects the target classes.
- **tol_norm** — centering tolerance as a fraction of the image (normalized);
  e.g. 0.05 = within 5% of center counts as centered.
- **dead-reckon** — moving by recorded odometry offsets (no vision), used to get
  the table into the down-cam frame before honing.
- **collection loop (S6)** — grab + place repeated for up to two objects; the
  second pass excludes the object already placed (`grabbed_label`). Grabbing one
  object still scores, so a missed second object is tolerated.
- **capstone / full mission** — `OctagonMission`, the single tree that chains
  S1→S7. Its default run does the whole task; no flags required.
- **score_level** — a 0–6 dial (`--score-level`) that runs every phase up to that
  number, then stops — for stepping up incrementally during bring-up.
- **do_pinger** — whether the full run starts with S1 acoustic homing (default) or
  skips it (`--no-pinger`) because the sub is hand-placed over the octagon.
