#!/usr/bin/env python3
"""
One USB camera out over ROS, for the detector, a recorder or a viewer to share.

  out  image_raw  sensor_msgs/Image, bgr8, turned by rotate degrees

The device is opened the way front_cam_driver.py opened it on dean-last-testing,
the last code to run properly on the boat: V4L2, then the size set, leaving the
pixel format to OpenCV. A read that fails is tried again; two seconds without a
frame ends the node, and launch restarts it to reopen the device. The day before
those runs, camera.py's MJPG 1080p lost the camera off USB within a minute,
though a cable or power could also explain that.

rotate turns each frame before it goes out. That driver turned the front camera
180 degrees, so vision_node has to be given the same rotate, to turn the
calibration with it.
"""

import signal
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

ROTATIONS = {0: None, 180: cv2.ROTATE_180}
GIVE_UP_AFTER = 2.0


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        device = self.declare_parameter("device", "/dev/video0").value
        width = self.declare_parameter("width", 640).value
        height = self.declare_parameter("height", 360).value
        rotate = self.declare_parameter("rotate", 0).value
        self.frame_id = self.declare_parameter("frame_id", "camera_optical_frame").value
        if rotate not in ROTATIONS:
            self.get_logger().error(f"rotate must be one of {list(ROTATIONS)}")
            raise SystemExit(1)
        self.rotation = ROTATIONS[rotate]

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "image_raw", 10)
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.last_frame = time.monotonic()
        if not self.capture.isOpened():
            self.get_logger().error(
                f"could not open {device}: check it is plugged in with "
                f"v4l2-ctl --list-devices, and that nothing else holds it with "
                f"fuser -v {device}",
            )
            raise SystemExit(1)

        fourcc = int(self.capture.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little")
        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"{device} as {self.frame_id}: "
            f"{actual_width}x{actual_height} {fourcc.decode()}, rotated {rotate}",
        )

    def step(self):
        """One frame out. False once the camera has given none for a while."""
        ok, frame = self.capture.read()
        if not ok:
            time.sleep(0.01)
            return time.monotonic() - self.last_frame < GIVE_UP_AFTER
        self.last_frame = time.monotonic()
        if self.rotation is not None:
            frame = cv2.rotate(frame, self.rotation)
        image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.frame_id
        self.image_pub.publish(image)
        return True


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    node = CameraNode()
    try:
        while not stop.is_set():
            if node.step():
                rclpy.spin_once(node, timeout_sec=0.0)
            elif not stop.is_set():
                # Nonzero, so launch respawns this and the device is reopened.
                node.get_logger().error(
                    f"no frame for {GIVE_UP_AFTER} s, reopening the camera",
                )
                raise SystemExit(1)
    finally:
        node.capture.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
