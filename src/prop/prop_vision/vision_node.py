#!/usr/bin/env python3
"""
One camera's frames, undistorted and run through the detector, out over ROS.

  in   image_topic  sensor_msgs/Image, camera_node's image_raw on the boat
  out  detections   yolo_msgs/DetectionArray, in the undistorted image's pixels
       camera_info  sensor_msgs/CameraInfo of that undistorted image
       image_rect   sensor_msgs/Image, the undistorted image, with publish_image

The boat has two, front and back, each its own node in its own namespace
(cameras.launch.py). frame_id is what tells them apart downstream: it has to be
that camera's optical frame in sensors.yaml, or its detections get projected as
if they came from the other end of the boat.

The pipeline is main.py's, minus the range estimate; the lidar has that.
"""

import signal
import threading
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from tracking import Tracker
from ultralytics import YOLO
from undistort import Undistorter
from yolo_msgs.msg import Detection, DetectionArray

HERE = Path(__file__).resolve().parent

# The detector can be slower than the camera. Frames left waiting in a queue
# would only eat into fusion's max_age, so it always takes the newest one.
NEWEST_FRAME = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def camera_info(intrinsics):
    """
    The undistorted image's calibration: distortion already removed, so a
    pinhole, and the crop already folded into the principal point.
    """
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    info = CameraInfo()
    info.width, info.height = intrinsics["width"], intrinsics["height"]
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def detection(track):
    x1, y1, x2, y2 = track["box"]
    msg = Detection()
    msg.class_id = track["class_id"]
    msg.class_name = track["label"]
    msg.score = track["conf"]
    msg.id = str(track["track_id"])
    msg.bbox.center.position.x = (x1 + x2) / 2.0
    msg.bbox.center.position.y = (y1 + y2) / 2.0
    msg.bbox.size.x = float(x2 - x1)
    msg.bbox.size.y = float(y2 - y1)
    return msg


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision")
        image_topic = self.declare_parameter("image_topic", "image_raw").value
        self.frame_id = self.declare_parameter("frame_id", "camera_optical_frame").value
        model = self.declare_parameter("model", str(HERE / "models" / "best.pt")).value
        calibration = self.declare_parameter(
            "calibration",
            str(HERE / "camera_params.json"),
        ).value
        conf = self.declare_parameter("conf", 0.25).value
        self.deck_line = self.declare_parameter("deck_line", 1.0).value
        publish_image = self.declare_parameter("publish_image", False).value
        # The rotate camera_node turned these frames by, so the calibration turns too.
        rotate = self.declare_parameter("rotate", 0).value

        self.undistorter = Undistorter(calibration, rotate=rotate)
        self.tracker = Tracker(YOLO(model), conf=conf)
        self.bridge = CvBridge()
        self.info = None

        self.detections_pub = self.create_publisher(DetectionArray, "detections", 10)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", 10)
        self.image_pub = None
        if publish_image:
            self.image_pub = self.create_publisher(
                Image,
                "image_rect",
                qos_profile_sensor_data,
            )
        self.create_subscription(Image, image_topic, self.on_image, NEWEST_FRAME)

        self.get_logger().info(
            f"{image_topic} as {self.frame_id}, detecting with {model}",
        )

    def above_deck(self, track, image_height):
        """
        Whether a box stays above deck_line, a fraction of the image's height.

        The model sees buoys in the boat's own deck fittings, and fusion pairs by
        bearing alone, so those boxes would vote for whatever lies along it.
        """
        return track["box"][3] <= self.deck_line * image_height

    def on_image(self, msg):
        rectified = self.undistorter.apply(self.bridge.imgmsg_to_cv2(msg, "bgr8"))
        if self.info is None:
            self.info = camera_info(self.undistorter.intrinsics)
            self.info.header.frame_id = self.frame_id

        detections = DetectionArray()
        detections.header.stamp = msg.header.stamp
        detections.header.frame_id = self.frame_id
        detections.detections = [
            detection(track)
            for track in self.tracker.track(rectified)
            if self.above_deck(track, rectified.shape[0])
        ]

        self.info.header.stamp = msg.header.stamp
        self.info_pub.publish(self.info)
        self.detections_pub.publish(detections)

        if self.image_pub is not None:
            image = self.bridge.cv2_to_imgmsg(rectified, "bgr8")
            image.header = detections.header
            self.image_pub.publish(image)


def main():
    # Ctrl-C lands mid-frame, usually inside the model. rclpy's own handler
    # would invalidate the context before that frame's publish, so stop
    # between frames instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    node = VisionNode()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
