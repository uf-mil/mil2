#!/usr/bin/env python3

# read opencv stream and publish as a ros msg
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from camera import Camera
from cv_bridge import CvBridge

# from image_transport_py import ImageTransport
from rclpy.node import Node
from sensor_msgs.msg import Image
from utils import load_config


class FrameCaptureNode(Node):
    def __init__(self):
        super().__init__("frame_capture_node")

        default_config = os.path.join(
            get_package_share_directory("object_classifier"),
            "config",
            "cam.yaml",
        )
        self.declare_parameter("config_file", default_config)
        cfg = load_config(self.get_parameter("config_file").value)

        cfg = cfg["/frame_capture_node"]["ros__parameters"]

        self.declare_parameter(
            "camera_source",
            "usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0",
        )

        self.declare_parameter("width", cfg["calibrated_width"])
        self.declare_parameter("height", cfg["calibrated_height"])
        self.declare_parameter("fps", cfg.get("fps", 30))
        self.declare_parameter("frame_id", cfg.get("frame_id", "camera"))

        source = self.get_parameter("camera_source").value
        if source == "":
            source = cfg["camera_index"]
        source = self.get_parameter("camera_source").get_parameter_value().string_value

        fps = self.get_parameter("fps").value
        self.frame_id = self.get_parameter("frame_id").value
        print(source)
        self.cam = Camera(
            source,
            width=self.get_parameter("width").value,
            height=self.get_parameter("height").value,
            fps=fps,
        )

        # self.image_transport = ImageTransport(
        #   "frame_capture_it", image_transport="compressed"
        # )
        self.img_pub = self.create_publisher(Image, "cam/image/raw", 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.publish_image)

    def publish_image(self):
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().warn("failed to read frame", throttle_duration_sec=2.0)
            return
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        self.img_pub.publish(image_msg)

    def destroy_node(self):
        self.cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrameCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
