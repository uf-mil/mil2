#!/usr/bin/env python3
"""Finds red and green lights in the front and down cameras by HSV threshold.

Publishes subjugator_msgs/LightDetections on light_detector/front and
light_detector/down. Positions are normalised to -1..1 (0 = image centre,
+x right, +y down) so consumers never need the image size.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from subjugator_msgs.msg import LightDetection, LightDetections

# OpenCV hue runs 0-179 and red wraps around 0.
HUE_RANGES = {"red": [(0, 10), (170, 179)], "green": [(40, 85)]}


class LightDetector(Node):
    def __init__(self):
        super().__init__("light_detector")
        self.declare_parameter("min_saturation", 100)
        self.declare_parameter("min_value", 150)
        self.declare_parameter("min_area", 50)
        self.bridge = CvBridge()
        self.front_pub = self.create_publisher(
            LightDetections,
            "light_detector/front",
            10,
        )
        self.down_pub = self.create_publisher(
            LightDetections,
            "light_detector/down",
            10,
        )
        self.create_subscription(
            Image,
            "front_cam/image_raw",
            lambda m: self.detect(m, self.front_pub),
            10,
        )
        self.create_subscription(
            Image,
            "down_cam/image_raw",
            lambda m: self.detect(m, self.down_pub),
            10,
        )

    def detect(self, msg, pub):
        hsv = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        out = LightDetections(header=msg.header)
        for color, ranges in HUE_RANGES.items():
            for cx, cy in self.blobs(hsv, ranges):
                out.lights.append(
                    LightDetection(
                        color=color,
                        x=2 * cx / width - 1,
                        y=2 * cy / height - 1,
                    ),
                )
        pub.publish(out)

    def blobs(self, hsv, ranges):
        min_s = self.get_parameter("min_saturation").value
        min_v = self.get_parameter("min_value").value
        min_area = self.get_parameter("min_area").value
        mask = np.bitwise_or.reduce(
            [cv2.inRange(hsv, (lo, min_s, min_v), (hi, 255, 255)) for lo, hi in ranges],
        )
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        return [
            tuple(centroids[i])
            for i in range(1, count)
            if stats[i, cv2.CC_STAT_AREA] >= min_area
        ]


def main():
    rclpy.init()
    rclpy.spin(LightDetector())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
