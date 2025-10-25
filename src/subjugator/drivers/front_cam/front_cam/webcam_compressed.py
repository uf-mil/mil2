#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")

        # Parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("publish_rate", 30.0)

        camera_index = self.get_parameter("camera_index").value
        publish_rate = self.get_parameter("publish_rate").value

        # Publishers
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            "/camera/image/compressed",
            10,
        )

        # CV and bridge setup
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open webcam index {camera_index}")
            exit(1)
        self.bridge = CvBridge()

        # Timer for loop
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Publishing webcam frames at {publish_rate:.1f} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame from webcam")
            return

        # Publish raw image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)

        # Publish compressed image (JPEG)
        compressed_msg = CompressedImage()
        compressed_msg.header = img_msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = cv2.imencode(".jpg", frame)[1].tobytes()
        self.compressed_pub.publish(compressed_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
