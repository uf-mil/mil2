# subjugator_bringup
This folder holds launch files and high level utilities.

## Running a task in sim: `run_task`

    ./run_task 5                  # full Task 5 mission, with a report
    ./run_task 5 --stage calib    # shortest stage; good first run
    ./run_task --list             # what is wired, with each stage's budget
    ./run_task 5 --dry-run        # print the plan without launching anything

One deterministic command for a whole sim run: bring-up, readiness gates,
settle-and-anchor, mission, teardown, then a report covering outcome, per-stage
timing, vehicle motion, perception health, and an independent ground-truth
verdict. Artifacts land in `pool_tests/task_runs/`. Exit codes are 0 mission
SUCCESS, 1 mission failed, 2 the harness itself broke.

`ros2 run subjugator_bringup run_task 5` is the same program; the repo-root
`./run_task` wrapper just sources the workspace first. See
[`pool_tests/SIM_TESTING.md`](pool_tests/SIM_TESTING.md) for the manual sequence
it automates.

## Down-cam centering (CenterCamera / HoneOverTarget) in sim

The `mission_planner` down-cam subscribers default to the **real-robot** topics,
so they must be overridden to run the down-cam centering (the `HoneOverTarget`
subtree / `CenterCamera` node) in simulation:

- **Image:** override `down_image_topic:=/down_cam/image_raw` (the gz->ROS bridge
  topic from `config/subjugator_bridge.yaml`). The default `/down_camera/rgb/image_raw`
  is the real driver's topic and has no publisher in sim.
- **Detections:** launch a second YOLO node on the down cam with
  `input_image_topic:=/down_cam/image_raw namespace:=yolo_down`. That publishes
  on `/yolo_down/detections`, which already matches the `down_detect_topic`
  default — no override needed.
- **Model:** centering only actually converges once the down YOLO loads a model
  whose class matches `CenterCamera`'s `label` (default `table`). Until then the
  topics connect but no `table` detection is produced, so the subtree degrades to
  its `AlwaysSuccess` fallback.
