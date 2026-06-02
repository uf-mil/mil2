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
        heading = self.find_path_marker_heading(img)
        if heading is not None:
            quat_msg = QuaternionStamped()
            quat_msg.header.stamp = self.get_clock().now().to_msg()
            quat_msg.header.frame_id = "down_cam"
            quat_msg.quaternion.x = 0.0
            quat_msg.quaternion.y = 0.0
            #   quat_msg.quaternion.z = math.cos(heading_rad / 2)
            #   quat_msg.quaternion.w = math.sin(heading_rad / 2)  # Yaw rotation
            self.heading_pub.publish(quat_msg)
            self.get_logger().info(
                f"Published path marker heading: {math.degrees(heading):.1f} degrees",
            )

    def find_path_marker_heading(self, img):
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
        # M = cv2.moments(largest_contour)
        # if M["m00"] == 0:
        #    return None

        # cx = int(M["m10"] / M["m00"])
        # cy = int(M["m01"] / M["m00"])

        # Calculate heading from center of image to contour center
        # img_center_x = img.shape[1] / 2
        # img_center_y = img.shape[0]

        rect = cv2.minAreaRect(largest_contour)
        (cx, cy), (w, h), rect_angle = rect

        # minAreaRect's angle refers to the rectangle's long axis
        # If width < height the long axis is vertical, so we adjust by 90°
        if w < h:
            rect_angle += 90.0

        # rect_angle is now the angle of the marker's long axis in image space
        # in degrees, measured clockwise from the image's horizontal axis

        yaw_deg = rect_angle
        yaw_rad = math.radians(yaw_deg)

        current_yaw = 0.0  # replace with real odomentry yaw
        option_a = yaw_rad
        option_b = yaw_rad + math.pi

        def angle_diff(a, b):
            return math.atan2(math.sin(a - b), math.cos(a - b))

        if abs(angle_diff(option_a, current_yaw)) < abs(
            angle_diff(option_b, current_yaw),
        ):
            return option_a
            # heading_rad = option_a
        else:
            return option_b
            # heading_rad = option_b

        # qw = math.cos(heading_rad / 2)
        # qz = math.sin(heading_rad / 2)

        # call the function to publish the heading as a quaternion already done in image call back, either call image call back in this function or vice versa
        # publish result in this function
