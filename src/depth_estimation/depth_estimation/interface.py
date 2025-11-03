# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .midas import midas_infer


class Cam(Enum):
    FRONT_CAM = 0
    DOWN_CAM = 1


class DepthNode(Node):

    def __init__(self):
        super().__init__("depth_node")

        self.front_cam_sub = self.create_subscription(
            Image,
            "/front_cam/image_raw",
            lambda img: self.listener_callback(img, Cam.FRONT_CAM),  # type: ignore[arg-type]
            10,
        )
        self.down_cam_sub = self.create_subscription(
            String,
            "/down_cam/image_raw",
            lambda img: self.listener_callback(img, Cam.DOWN_CAM),  # type: ignore
            10,
        )
        print("Listening for image data..")
        self.front_cam_pub = self.create_publisher(Image, "/front_cam/image_depth", 10)
        self.down_cam_pub = self.create_publisher(Image, "/down_cam/image_depth", 10)
        self.bridge = CvBridge()

    def listener_callback(self, img: Image, cam: Cam):
        inference = midas_infer(self.bridge.imgmsg_to_cv2(img))
        if cam == Cam.FRONT_CAM:
            self.front_cam_pub.publish(self.bridge.cv2_to_imgmsg(inference))
        else:
            self.down_cam_pub.publish(self.bridge.cv2_to_imgmsg(inference))


def main(args=None):
    rclpy.init(args=args)

    depth_node = DepthNode()

    rclpy.spin(depth_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
