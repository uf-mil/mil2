#!/usr/bin/env python3
"""
Path marker heading detector.

Subscribes to /down_cam/image_raw, finds the orange path marker,
and publishes its heading as a quaternion to /path_marker/heading.

Run this after the sub is positioned directly above the marker.
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class PathMarkerHeading(Node):
    def __init__(self):
        super().__init__("path_marker_heading")

        self.bridge = CvBridge()
        # Publisher for the path marker heading as a quaternion
        self.heading_pub = self.create_publisher(
            QuaternionStamped,
            "/path_marker/heading",
            10,
        )
        # Subscribe to the downward camera
        self.create_subscription(Image, "/down_cam/image_raw", self.image_callback, 10)
        self.get_logger().info(
            "PathMarkerHeading: waiting for images on /down_cam/image_raw",
        )

    def image_callback(self, msg: Image):
        # convert ROS image to OpenCV format
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        heading_rad = self.find_path_marker_heading(img)
        if heading_rad is None:
            return
        qw = math.cos(heading_rad / 2.0)
        qz = math.sin(heading_rad / 2.0)

        quat_msg = QuaternionStamped()
        quat_msg.header.stamp = self.get_clock().now().to_msg()
        quat_msg.header.frame_id = "odom"
        quat_msg.quaternion.x = 0.0
        quat_msg.quaternion.y = 0.0
        quat_msg.quaternion.z = qz
        quat_msg.quaternion.w = qw

        self.heading_pub.publish(quat_msg)
        self.get_logger().info(
            f"Published path marker heading: {math.degrees(heading_rad):.1f} degrees",
        )

    def find_path_marker_heading(self, img):
        """
        Takes an OpenCV image, finds the orange path marker,
        and returns its heading as a yaw angle in radians.
        Returns None if no marker is found.
        """
        # Convert to HSV and threshold for orange color
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 100, 100])  # Need to adjust
        upper_orange = np.array([15, 255, 255])  # need to adjust
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours and get the largest one because that's most likely the marker
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn("No path marker detected")
            return None

        largest_contour = max(contours, key=cv2.contourArea)

        # Filter out small contours that are unlikely to be the marker
        if cv2.contourArea(largest_contour) < 1000:
            self.get_logger().warn("Largest orange blob too small, ignoring")
            return None

        rect = cv2.minAreaRect(largest_contour)
        (cx, cy), (w, h), rect_angle = rect

        # minAreaRect's angle refers to the rectangle's long axis
        # If width < height the long axis is vertical, so we adjust by 90°
        if w < h:
            rect_angle += 90.0

        # rect_angle is now the angle of the marker's long axis in image space
        # in degrees, measured clockwise from the image's horizontal axis

        yaw_rad = math.radians(rect_angle)

        # fix 180° ambiguity using current heading as a tiebreaker
        current_yaw = 0.0  # replace with real odomentry yaw
        option_a = yaw_rad
        option_b = yaw_rad + math.pi

        def angle_diff(a, b):
            return math.atan2(math.sin(a - b), math.cos(a - b))

        if abs(angle_diff(option_a, current_yaw)) < abs(
            angle_diff(option_b, current_yaw),
        ):
            return option_a
        else:
            return option_b


def main():
    rclpy.init()
    node = PathMarkerHeading()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
