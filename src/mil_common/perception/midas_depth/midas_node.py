import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
from midas.dpt_depth import DPTDepthModel  # depending on MiDaS version

class MidasDepthNode(Node):
    def __init__(self):
        super().__init__('midas_depth_node')
        self.bridge = CvBridge()

        # Load MiDaS model
        self.model = torch.hub.load("intel-isl/MiDaS", "DPT_Large")
        self.model.eval()
        self.transform = torch.hub.load("intel-isl/MiDaS", "transforms").dpt_transform

        # Subscribers
        self.front_sub = self.create_subscription(Image, '/front_cam/image_raw', self.callback_front, 10)
        self.down_sub = self.create_subscription(Image, '/down_cam/image_raw', self.callback_down, 10)

        # Publishers
        self.front_pub = self.create_publisher(Image, '/front_cam/image_depth', 10)
        self.down_pub = self.create_publisher(Image, '/down_cam/image_depth', 10)

    def process_image(self, msg, publisher):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        input_batch = self.transform(frame).to('cuda' if torch.cuda.is_available() else 'cpu')
        with torch.no_grad():
            prediction = self.model(input_batch)
            depth_map = prediction.squeeze().cpu().numpy()

        depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
        depth_uint8 = depth_normalized.astype(np.uint8)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding='mono8')
        publisher.publish(depth_msg)

    def callback_front(self, msg):
        self.process_image(msg, self.front_pub)

    def callback_down(self, msg):
        self.process_image(msg, self.down_pub)


def main(args=None):
    rclpy.init(args=args)
    node = MidasDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
