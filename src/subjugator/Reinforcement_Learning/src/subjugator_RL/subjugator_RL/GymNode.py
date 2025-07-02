import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from subjugator_RL.locks import cam_lock, imu_lock


class GymNode(Node):
    def __init__(self):
        super().__init__("GymNode")
        self.cvBridge = CvBridge()
        # self.subEnv = SubEnv(movement_publisher=self.publish_action_as_wrench)

        # imu attributes initalized here
        self.imu_subscription = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.imu_callback,
            10,  # topic for sub9 cam
        )
        self.imu_data = None

        # cam attributes initialized here
        self.cam_subscription = self.create_subscription(
            Image,
            "/front_cam/image_raw",
            self.image_callback,
            10,  # topic for sub9 cam
        )
        self.cam_data = None

        # publisher for sub motion
        self.wrench_publisher = self.create_publisher(Wrench, "cmd_wrench", 10)

    def image_callback(self, msg):
        time.sleep(0.5)  # Delay for image callback
        self.get_logger().info("Image received")

        # # Code for debugging
        # cv2.destroyAllWindows()
        # # Front cam image size is 600x960, before max pooling
        # cv2.imshow("Front cam", cvBridge.imgmsg_to_cv2(msg, "bgr8"))
        # cv2.waitKey(40)

        # Simple max pooling algorithm for image
        pool_size = 12
        # Divide original image dimensions by pool size to get pooled image dimensions
        net = cv2.dnn.Net()
        params = {
            "kernel_w": pool_size,
            "kernel_h": pool_size,
            "stride_w": pool_size,
            "stride_h": pool_size,
            "pool": "max",
        }
        net.addLayerToPrev("pool", "Pooling", cv2.CV_32F, params)

        image = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        net.setInput(cv2.dnn.blobFromImage(image))
        out = net.forward()
        pooled_image = cv2.dnn.imagesFromBlob(out)[0].astype("uint8")

        # # Code for debugging
        # cv2.imshow("Pooled image", pooled_image)
        # cv2.waitKey(20)
        with cam_lock:
            self.cam_data = pooled_image

    def imu_callback(self, msg):
        time.sleep(0.1)  # Unblock thread so other things can run
        self.get_logger().info("Imu received")

        # lock thread
        with imu_lock:
            self.imu_data = msg
        # release lock

    def publish_action_as_wrench(self, action):
        force_action = action[:3]
        torque_action = action[3:6]

        wrench_msg = Wrench()

        wrench_msg.force = Vector3(
            x=float(force_action[0]),
            y=float(force_action[1]),
            z=float(force_action[2]),
        )

        wrench_msg.torque = Vector3(
            x=float(torque_action[0]),
            y=float(torque_action[1]),
            z=float(torque_action[2]),
        )
        
        self.wrench_publisher.publish(wrench_msg)


def safe_rclpy_init():
    try:
        rclpy.init()
    except RuntimeError:
        pass


safe_rclpy_init()
