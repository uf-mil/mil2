#!/usr/bin/env python3
"""
sim_detection_publisher.py

Projects competition element 3D bounding boxes into camera image space using
ground-truth Gazebo poses. Two modes (set via ROS parameters):

  publish_detections=True  — publishes DetectionArray at 30 Hz so missions
                              can run without YOLO or camera rendering
  collect_data=True        — saves each incoming camera frame as a PNG with a
                              YOLO-format .txt annotation file alongside it
"""

import os
import threading

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from yolo_msgs.msg import Detection, DetectionArray

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SimDetectionPublisher(Node):
    def __init__(self):
        super().__init__("sim_detection_publisher")

        self.declare_parameter("config_file", "")
        self.declare_parameter("publish_detections", True)
        self.declare_parameter("collect_data", False)
        self.declare_parameter("camera_topic", "/front_cam")
        self.declare_parameter("output_dir", os.path.expanduser("~/sim_training_data"))
        self.declare_parameter("save_every_n_frames", 10)

        config_path = self.get_parameter("config_file").value
        self.publish_detections = self.get_parameter("publish_detections").value
        self.collect_data = self.get_parameter("collect_data").value
        camera_topic = self.get_parameter("camera_topic").value
        self.output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        self._save_every_n = self.get_parameter("save_every_n_frames").value
        self._frame_count = 0

        if not config_path:
            config_path = os.path.join(
                get_package_share_directory("subjugator_vision"),
                "config",
                "sim_objects.yaml",
            )

        with open(config_path) as f:
            config = yaml.safe_load(f)

        self.objects = config["objects"]
        self.camera_link = config["camera_link"]
        self._model_name = config.get("model_name", "sub9")
        gz_pose_topic = config["gz_pose_topic"]

        # Camera intrinsics — populated from /camera_info on first message
        self.fx = self.fy = self.cx = self.cy = None
        self.img_width = self.img_height = None

        # Rotation from camera link frame to optical (image) frame.
        # Camera link: X=forward, Y=left, Z=up  (robot convention, rpy=0 on sub)
        # Optical frame: X=right, Y=down, Z=forward
        self._R_opt = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)

        # World-to-camera transform updated by the Gazebo pose callback
        self._pose_lock = threading.Lock()
        self._T_world_to_cam: np.ndarray | None = None

        self.create_subscription(
            CameraInfo,
            f"{camera_topic}/camera_info",
            self._camera_info_cb,
            SENSOR_QOS,
        )

        if self.collect_data:
            os.makedirs(self.output_dir, exist_ok=True)
            self._bridge = CvBridge()
            self.create_subscription(
                Image,
                f"{camera_topic}/image_raw",
                self._image_cb,
                SENSOR_QOS,
            )
            self.get_logger().info(f"collect_data mode: saving to {self.output_dir}")

        if self.publish_detections:
            self._det_pub = self.create_publisher(
                DetectionArray,
                "/yolo/detections",
                10,
            )
            self.create_timer(1.0 / 30.0, self._publish_timer_cb)
            self.get_logger().info(
                "publish_detections mode: publishing on /yolo/detections",
            )

        # Gazebo subscriber runs callbacks on gz's own thread
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, gz_pose_topic, self._gz_pose_cb)

        self.get_logger().info(
            f"Watching {len(self.objects)} objects, camera link: {self.camera_link}",
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        first = self.fx is None
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.img_width = msg.width
        self.img_height = msg.height
        if first:
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.1f} cx={self.cx} {self.img_width}x{self.img_height}",
            )

    def _gz_pose_cb(self, msg: Pose_V):
        # dynamic_pose/info publishes the sub model in world frame, but nested
        # link poses (like front_cam_link) are relative to that model.
        # We must compose: T_cam_world = T_model_world @ T_cam_in_model.
        model_pose = None
        cam_pose = None
        for pose in msg.pose:
            if pose.name == self._model_name:
                model_pose = pose
            elif pose.name == self.camera_link:
                cam_pose = pose
            if model_pose is not None and cam_pose is not None:
                break

        if model_pose is None or cam_pose is None:
            return

        p, q = model_pose.position, model_pose.orientation
        T_model_in_world = self._make_transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        p, q = cam_pose.position, cam_pose.orientation
        T_cam_in_model = self._make_transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        T_cam_in_world = T_model_in_world @ T_cam_in_model
        T_world_to_link = self._invert(T_cam_in_world)

        T = np.eye(4)
        T[:3, :3] = self._R_opt @ T_world_to_link[:3, :3]
        T[:3, 3] = self._R_opt @ T_world_to_link[:3, 3]
        with self._pose_lock:
            self._T_world_to_cam = T

    def _publish_timer_cb(self):
        with self._pose_lock:
            T = self._T_world_to_cam
        if T is None or self.fx is None:
            return

        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_link

        for obj in self.objects:
            bbox = self._project(obj["position"], obj["half_extents"], T)
            if bbox is not None:
                msg.detections.append(self._build_detection(obj, bbox))

        self._det_pub.publish(msg)

    def _image_cb(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % self._save_every_n != 0:
            return

        with self._pose_lock:
            T = self._T_world_to_cam
        if T is None or self.fx is None:
            self.get_logger().warn(
                f"image received but not ready: T={'set' if T is not None else 'None'}, fx={self.fx}",
                throttle_duration_sec=2.0,
            )
            return

        stamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        lines = []

        for obj in self.objects:
            bbox = self._project(obj["position"], obj["half_extents"], T)
            if bbox is None:
                continue
            cx, cy, w, h = bbox
            lines.append(
                f"{obj['class_id']} "
                f"{cx / self.img_width:.6f} "
                f"{cy / self.img_height:.6f} "
                f"{w / self.img_width:.6f} "
                f"{h / self.img_height:.6f}",
            )

        if not lines:
            return

        img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(os.path.join(self.output_dir, f"{stamp}.png"), img)
        with open(os.path.join(self.output_dir, f"{stamp}.txt"), "w") as f:
            f.write("\n".join(lines))

        self.get_logger().info(f"Saved {stamp}.png  ({len(lines)} annotations)")

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _make_transform(self, x, y, z, qx, qy, qz, qw) -> np.ndarray:
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        T[:3, :3] = np.array(
            [
                [
                    1 - 2 * (qy**2 + qz**2),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx**2 + qz**2),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx**2 + qy**2),
                ],
            ],
        )
        return T

    def _invert(self, T: np.ndarray) -> np.ndarray:
        R, t = T[:3, :3], T[:3, 3]
        Ti = np.eye(4)
        Ti[:3, :3] = R.T
        Ti[:3, 3] = -R.T @ t
        return Ti

    def _project(self, position, half_extents, T_world_to_cam):
        """Return (cx, cy, w, h) in pixels, or None if not visible."""
        ox, oy, oz = position
        dx, dy, dz = half_extents

        corners = np.array(
            [
                [ox + sx * dx, oy + sy * dy, oz + sz * dz, 1.0]
                for sx in (-1, 1)
                for sy in (-1, 1)
                for sz in (-1, 1)
            ],
        )

        p_cam = (T_world_to_cam @ corners.T).T
        visible = p_cam[:, 2] > 0.01
        if not np.any(visible):
            return None
        p_cam = p_cam[visible]

        u = self.fx * p_cam[:, 0] / p_cam[:, 2] + self.cx
        v = self.fy * p_cam[:, 1] / p_cam[:, 2] + self.cy

        x0 = float(np.clip(u.min(), 0, self.img_width))
        x1 = float(np.clip(u.max(), 0, self.img_width))
        y0 = float(np.clip(v.min(), 0, self.img_height))
        y1 = float(np.clip(v.max(), 0, self.img_height))

        if x1 <= x0 or y1 <= y0:
            return None

        return (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0

    def _build_detection(self, obj, bbox) -> Detection:
        cx, cy, w, h = bbox
        det = Detection()
        det.class_id = obj["class_id"]
        det.class_name = obj["class_name"]
        det.score = 1.0
        det.bbox.center.position.x = cx
        det.bbox.center.position.y = cy
        det.bbox.size.x = w
        det.bbox.size.y = h
        return det


def main():

    rclpy.init()
    node = SimDetectionPublisher()
    try:
        while rclpy.ok():
            # spin_once with a short timeout releases the Python GIL while
            # waiting, giving gz transport callbacks a window to execute.
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
