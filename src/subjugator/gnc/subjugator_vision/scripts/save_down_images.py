#!/usr/bin/env python3
"""
Continuously save frames from /down_cam/image_raw for building a YOLO dataset.

Saves one out of every `save_every` frames (default 15, ~2 img/s at 30 fps) so
the dataset isn't thousands of near-identical frames. Runs until Ctrl-C.
"""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DownCamSaver(Node):
    def __init__(self):
        super().__init__("down_cam_saver")

        default_dir = os.path.join(os.path.expanduser("~"), "sim_images/down")
        self.declare_parameter("save_dir", default_dir)
        self.declare_parameter("save_every", 15)

        self.save_every = self.get_parameter("save_every").value
        self.count = 0

        topic = "/down_cam/image_raw"
        self.dir = os.path.abspath(
            os.path.expanduser(self.get_parameter("save_dir").value),
        )

        os.makedirs(self.dir, exist_ok=True)
        self.bridge = CvBridge()

        self.create_subscription(Image, topic, self.callback, 10)
        self.get_logger().info(
            f"DownCamSaver: waiting for image on {topic}, saving to {self.dir}",
        )

    def callback(self, msg: Image):
        self.count += 1
        if self.count % self.save_every != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # time for file name
        stamp = self.get_clock().now().to_msg()
        filename = f"down_cam_{stamp.sec}_{stamp.nanosec}.png"
        cv2.imwrite(os.path.join(self.dir, filename), img)

        self.get_logger().info(f"Saved {filename}")


def main():
    rclpy.init()
    node = DownCamSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
