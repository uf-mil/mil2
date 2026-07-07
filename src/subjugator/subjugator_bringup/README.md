# subjugator_bringup
This folder holds launch files and high level utilities.

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
