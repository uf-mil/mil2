# Pool tests — Task 5 down-cam stack (S2 + S3)

Guided poolside runner for the down-cam centering + target-selection stack.
Run it on the sub's onboard machine. See the design spec at
`docs/superpowers/specs/2026-06-25-pool-test-protocol-design.md`.

## Run it (always under tmux so it survives disconnect)

```bash
tmux new -s pooltest
source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
cd src/subjugator/subjugator_bringup/pool_tests
./pooltest.sh                 # guided sequence
./pooltest.sh calib           # one stage
./pooltest.sh --help          # all stages + flags
```

**After you type `GO` you have 30 s (default) to disconnect and clear the sub.**
It then runs autonomously. Detach tmux with `Ctrl-b d`; reattach with
`tmux attach -t pooltest` to fill in the results form.

Rehearse the whole flow dry, with no hardware: add `--dry-run`. Rehearse in
Gazebo: add `--sim` (also launch a `yolo_down` node on `/down_cam/image_raw`).

## Roles

| role | object classes |
|------|----------------|
| `survey_repair` | nut_bolt, plug |
| `search_rescue` | pill, bandage |

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

## One-page checklist

| stage | POSITION the sub | TYPE | EXPECT | RECORD |
|-------|------------------|------|--------|--------|
| preflight | n/a | (none) | all topics live | MISSING items |
| calib | ~1 m over table, level | `GO` | self-centers & holds | converging signs + gains |
| hone | ~1 m over table | `GO` | holds center | confidence, altitude, stability |
| select | objects in down-cam view | `--role`, `GO` | `locked '<class>'` | locked label, conf, frames |
| combined | ~1 m over table, objects in view | `--role`, `GO` | centers then locks | label, total time, handoff y/n |
| deadreckon | drive to CP1..CP4 | `SNAP` x4 | constants printed | constants.txt values |
| full_s2 | S1-arrival start pose | `GO` | ends over table | which step failed |

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
