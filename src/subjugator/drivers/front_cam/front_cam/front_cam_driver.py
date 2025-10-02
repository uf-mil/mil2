import cv2
import rclpy
from cv2.typing import MatLike
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def rotate_front_cam(frame: MatLike) -> MatLike:
    # old code to add padding
    # PADDING = 0
    # print(rotation_angle)

    # Add black padding around the image
    # padded = cv2.copyMakeBorder(
    # frame,
    # top=PADDING,
    # bottom=PADDING,
    # left=PADDING,
    # right=PADDING,
    # borderType=cv2.BORDER_CONSTANT,
    # value=(0, 0, 0),  # black
    # )

    # magic numbers:
    rotation_angle = 180

    # Get new dimensions
    (h, w) = frame.shape[:2]
    center = (w // 2, h // 2)

    # Get rotation matrix
    M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)

    # Rotate the padded image
    rotated = cv2.warpAffine(frame, M, (w, h))
    return rotated


class FrontCamDriver(Node):
    def __init__(self):
        super().__init__("front_cam_driver")

        # say params exist
        self.declare_parameter("camera-id", "")
        self.declare_parameter("camera-topic", "front_cam/raw")

        # Get parameter values
        camera_id = self.get_parameter("camera-id").get_parameter_value().string_value
        topic_name = (
            self.get_parameter("camera-topic").get_parameter_value().string_value
        )

        self.get_logger().warn(camera_id)
        self.get_logger().warn(topic_name)

        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error("could not open camera")
            return

        # Create publisher
        self.img_pub = self.create_publisher(Image, topic_name, 10)

        # run this forever
        self.get_frame_and_publish()

    def get_frame_and_publish(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("no frame from camera")

            frame = rotate_front_cam(frame)

            # publish frame to "camera/rgb/camera_raw"
            try:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame)  # , encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.encoding = "bgr8"

                # do i need this??? TODO
                img_msg.header.frame_id = "idk the camera frame"

                self.img_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Error converting/publishing image: {e!s}")


def main():
    rclpy.init()
    _ = FrontCamDriver()
    # rclpy.spin(_) # this node spins itself
    rclpy.shutdown()


if __name__ == "__main__":
    main()
