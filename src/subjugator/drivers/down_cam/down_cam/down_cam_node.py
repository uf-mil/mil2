import cv2
import rclpy
from cv2.typing import MatLike
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def rotate_front_cam(frame: MatLike) -> MatLike:
    # magic numbers:
    PADDING = 100
    rotation_angle = 242

    # Add black padding around the image
    padded = cv2.copyMakeBorder(
        frame,
        top=PADDING,
        bottom=PADDING,
        left=PADDING,
        right=PADDING,
        borderType=cv2.BORDER_CONSTANT,
        value=(0, 0, 0),  # black
    )

    # Get new dimensions
    (h, w) = padded.shape[:2]
    center = (w // 2, h // 2)

    # Get rotation matrix
    M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)

    # Rotate the padded image
    rotated = cv2.warpAffine(padded, M, (w, h))
    return rotated


class DownCamDriver(Node):
    def __init__(self):
        super().__init__("front_cam_driver")

        cam_path = "/dev/v4l/by-id/usb-Chicony_Tech._Inc._Dell_Webcam_WB7022_77A8ADD45565-video-index0"

        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(cam_path)
        if not self.cap.isOpened():
            self.get_logger().error("could not open camera")
            return

        # Create publisher
        self.img_pub = self.create_publisher(Image, "down_camera/rgb/image_raw", 10)

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
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()

                # do i need this??? TODO
                img_msg.header.frame_id = "idk the camera frame"

                self.img_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Error converting/publishing image: {e!s}")


def main():
    rclpy.init()
    _ = DownCamDriver()
    # rclpy.spin(_) # this node spins itself
    rclpy.shutdown()


if __name__ == "__main__":
    main()
