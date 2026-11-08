#!/usr/bin/env python3
"""
Watches each buoy's Task 1 light in the cameras' raw frames.

  in   /<camera>/image_raw  sensor_msgs/Image, camera_node's frames
       /labelled_objects    mil_msgs/PerceptionObjectArray, from object_map
  out  /beacons             mil_msgs/PerceptionObjectArray, the buoy-sized
                            objects, labeled_classification set to their light:
                            a prop_vision.beacons state, or UNKNOWN
       ~/<camera>/debug_image  the windows and lights drawn, with debug_image
  srv  ~/reset              std_srvs/Trigger, forget every light seen so far

Every frame, each buoy within max_range is projected into the raw image,
distortion and all, as a window where its beacon should light up: beacon_low to
beacon_high above the objects' frame, beacon_width across. The raw frame has
about three times the pixels of the undistorted crop, and a beacon ten metres
off is only a few pixels wide. The window's colour that frame goes into the
buoy's history; a window partly off the image or behind a nearer buoy's is
skipped rather than counted as dark.

rotate must be what camera_node turned each camera's frames by, so the
calibration turns with them; cameras.launch.py passes both from one setting.

The lights only come on once the boat says it is attempting Task 1, so the
mission calls reset then: until it does, every close buoy votes OFF.
"""

import math
import signal
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mil_msgs.msg import PerceptionObject, PerceptionObjectArray
from prop_vision.beacons import Beacons, lit_color
from prop_vision.calibration import load
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

HERE = Path(__file__).resolve().parent
NEWEST_FRAME = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
UNKNOWN = "UNKNOWN"
# Off the optical axis, past which the lens model is unreliable.
MAX_ANGLE = math.radians(50.0)
DRAW = {
    "red": (0, 0, 255),
    "green": (0, 200, 0),
    "blue": (255, 90, 0),
    None: (170, 170, 170),
}


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
    )


def window_corners(x, y, camera_xy, low, high, width):
    """The corners, in the objects' frame, of the beacon window for a buoy at (x, y)."""
    toward = np.array([x, y]) - camera_xy
    across = np.array([-toward[1], toward[0]]) * (width / 2 / np.linalg.norm(toward))
    return np.array(
        [
            [x + s * across[0], y + s * across[1], z]
            for s in (-1, 1)
            for z in (low, high)
        ],
    )


def overlap(a, b):
    """How much of span a the span b covers."""
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


class Camera:
    def __init__(self, node, name, topic, rotate, debug):
        self.name = name
        self.optical_frame = f"{name}_optical_frame"
        self.rotate = rotate
        self.size = None
        self.matrix = self.dist = None
        self.debug_pub = None
        if debug:
            self.debug_pub = node.create_publisher(Image, f"~/{name}/debug_image", 1)
        node.create_subscription(
            Image,
            topic,
            lambda msg: node.on_image(self, msg),
            NEWEST_FRAME,
        )

    def calibrate(self, calibration, width, height):
        if self.size != (width, height):
            self.matrix, self.dist = load(calibration, width, height, self.rotate)
            self.size = (width, height)

    def windows(self, objects, rotation, translation, settings):
        """(distance, object, pixel box) for each buoy whose window is in view and not hidden."""
        camera_xy = (-rotation.T @ translation)[:2]
        width, height = self.size
        seen = []
        for obj in objects:
            p = obj.pose.position
            distance = float(np.hypot(p.x - camera_xy[0], p.y - camera_xy[1]))
            if not settings["min_range"] <= distance <= settings["max_range"]:
                continue
            corners = window_corners(p.x, p.y, camera_xy, *settings["window"])
            in_camera = corners @ rotation.T + translation
            depth = in_camera[:, 2]
            if (depth < 0.5).any():
                continue
            if (np.abs(in_camera[:, :2] / depth[:, None]) > math.tan(MAX_ANGLE)).any():
                continue
            pixels, _ = cv2.projectPoints(
                in_camera,
                np.zeros(3),
                np.zeros(3),
                self.matrix,
                self.dist,
            )
            left, top = pixels.reshape(-1, 2).min(axis=0)
            right, bottom = pixels.reshape(-1, 2).max(axis=0)
            box = (max(left, 0), max(top, 0), min(right, width), min(bottom, height))
            if (box[2] - box[0]) * (box[3] - box[1]) < 0.6 * (right - left) * (
                bottom - top
            ):
                continue
            seen.append((distance, obj, tuple(int(round(v)) for v in box)))

        kept = []
        for distance, obj, box in sorted(seen, key=lambda s: s[0]):
            span = (box[0], box[2])
            if all(
                overlap(span, (k[2][0], k[2][2])) <= 0.3 * (span[1] - span[0])
                for k in kept
            ):
                kept.append((distance, obj, box))
        return kept


class BeaconNode(Node):
    def __init__(self):
        super().__init__("beacons")
        names = list(self.declare_parameter("cameras", ["front_camera"]).value)
        topics = list(self.declare_parameter("image_topics", [""]).value)
        rotations = list(self.declare_parameter("rotate", [180]).value)
        self.calibration = self.declare_parameter(
            "calibration",
            str(HERE / "camera_params.json"),
        ).value
        self.max_size = self.declare_parameter("max_size", 1.2).value
        self.off_range = self.declare_parameter("off_range", 8.0).value
        self.settings = {
            "min_range": self.declare_parameter("min_range", 1.0).value,
            "max_range": self.declare_parameter("max_range", 20.0).value,
            "window": (
                self.declare_parameter("beacon_low", 0.2).value,
                self.declare_parameter("beacon_high", 0.9).value,
                self.declare_parameter("beacon_width", 0.7).value,
            ),
        }
        self.min_pixels = self.declare_parameter("min_pixels", 4).value
        self.max_fraction = self.declare_parameter("max_fraction", 0.2).value
        self.beacons = Beacons(
            window=self.declare_parameter("history", 4.0).value,
            min_watch=self.declare_parameter("min_watch", 2.5).value,
            min_votes=self.declare_parameter("min_votes", 15).value,
        )
        debug = self.declare_parameter("debug_image", False).value

        self.bridge = CvBridge()
        self.objects = []
        self.objects_frame = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PerceptionObjectArray, "/beacons", 10)
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.create_subscription(
            PerceptionObjectArray,
            "/labelled_objects",
            self.on_objects,
            10,
        )
        topics += [""] * (len(names) - len(topics))
        self.cameras = [
            Camera(self, name, topic or f"/{name}/image_raw", rotate, debug)
            for name, topic, rotate in zip(names, topics, rotations)
        ]
        self.get_logger().info(
            "watching beacons in "
            + ", ".join(f"{c.name} (rotated {c.rotate})" for c in self.cameras),
        )

    def on_reset(self, _, response):
        self.beacons.forget(set())
        self.get_logger().info("forgot every light seen so far")
        response.success = True
        return response

    def on_objects(self, msg):
        self.objects = [
            o for o in msg.objects if max(o.scale.x, o.scale.y) <= self.max_size
        ]
        if msg.objects:
            self.objects_frame = msg.objects[0].header.frame_id
        self.beacons.forget({o.id for o in msg.objects})

        out = PerceptionObjectArray()
        for o in self.objects:
            beacon = PerceptionObject()
            beacon.header = o.header
            beacon.id = o.id
            beacon.pose = o.pose
            beacon.scale = o.scale
            beacon.classification = o.labeled_classification
            beacon.labeled_classification = self.beacons.state(o.id) or UNKNOWN
            beacon.classification_confidence = self.beacons.confidence(o.id)
            out.objects.append(beacon)
        self.pub.publish(out)

    def on_image(self, camera, msg):
        if not self.objects or self.objects_frame is None:
            return
        transform = self.lookup(camera.optical_frame, msg.header.stamp)
        if transform is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        camera.calibrate(self.calibration, frame.shape[1], frame.shape[0])
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        drawn = []
        for distance, obj, (left, top, right, bottom) in camera.windows(
            self.objects,
            *transform,
            self.settings,
        ):
            color = lit_color(
                frame[top:bottom, left:right],
                self.min_pixels,
                self.max_fraction,
            )
            self.beacons.add(obj.id, color, t, close=distance <= self.off_range)
            drawn.append((obj.id, color, (left, top, right, bottom)))

        if camera.debug_pub is not None:
            for object_id, color, (left, top, right, bottom) in drawn:
                cv2.rectangle(frame, (left, top), (right, bottom), DRAW[color], 1)
                label = f"#{object_id} {self.beacons.state(object_id) or '?'}"
                cv2.putText(
                    frame,
                    label,
                    (left, max(top - 3, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    DRAW[color],
                    1,
                )
            out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            out.header = msg.header
            camera.debug_pub.publish(out)

    def lookup(self, target, stamp):
        """Rotation and translation taking the objects' frame into target."""
        try:
            tf = self.tf_buffer.lookup_transform(target, self.objects_frame, stamp)
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(target, self.objects_frame, Time())
            except TransformException as ex:
                self.get_logger().warn(
                    f"no transform {self.objects_frame} -> {target}: {ex}",
                    throttle_duration_sec=5.0,
                )
                return None
        t = tf.transform.translation
        return rotation_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z])


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = BeaconNode()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
