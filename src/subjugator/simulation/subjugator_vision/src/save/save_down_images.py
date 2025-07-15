#!/usr/bin/env python3
"""
Take a single image from /down_cam/image_raw, save it, and exit.
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DownCamSaver(Node):
    def __init__(self):
        super().__init__('down_cam_saver')

        default_dir = os.path.join(os.path.expanduser("~"), "sim_images/down")
        self.declare_parameter('save_dir', default_dir)

        topic = '/down_cam/image_raw'               
        self.dir = os.path.abspath(os.path.expanduser(self.get_parameter('save_dir').value))

        os.makedirs(self.dir, exist_ok=True)
        self.bridge = CvBridge()

        self.create_subscription(Image, topic, self.callback, 10)
        self.get_logger().info(f"DownCamSaver: waiting for image on {topic}, saving to {self.dir}")

    def callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        #time for file name
        stamp = self.get_clock().now().to_msg()
        filename = f"down_cam_{stamp.sec}_{stamp.nanosec}.png"
        cv2.imwrite(os.path.join(self.dir, filename), img)

        self.get_logger().info(f"Saved {filename}")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = DownCamSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
