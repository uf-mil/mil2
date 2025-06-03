#!/usr/bin/env python3
"""
Subscribes to a camera topic (default /front_cam/image_raw)
and writes every N-th frame to disk for training data.
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('image_topic', '/front_cam/image_raw')
        self.declare_parameter('save_dir', 'sim_images')   # will be created in CWD
        self.declare_parameter('every_n', 1)               # save every frame

        topic     = self.get_parameter('image_topic').get_parameter_value().string_value
        self.dir  = self.get_parameter('save_dir').get_parameter_value().string_value
        self.skip = self.get_parameter('every_n').get_parameter_value().integer_value

        os.makedirs(self.dir, exist_ok=True)
        self.bridge  = CvBridge()
        self.counter = 0

        self.create_subscription(Image, topic, self._callback, 10)
        self.get_logger().info(f'ImageSaver listening on {topic}, saving to {self.dir}')

    
    def _callback(self, msg: Image):
        self.counter += 1
        if self.counter % self.skip:
            return                                  # skip this frame

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        stamp   = self.get_clock().now().to_msg()
        fname   = f'{stamp.sec}_{stamp.nanosec}.png'
        abs_out = os.path.join(self.dir, fname)

        cv2.imwrite(abs_out, cv_img)
        self.get_logger().info(f'Saved {abs_out}')

def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
