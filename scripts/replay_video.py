import sys

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rclpy.init()

cv_bridge = CvBridge()
node = Node("replay_video")

pub = node.create_publisher(Image, "/front_cam/image_raw", 10)

cap = cv2.VideoCapture(sys.argv[1])
while cap.isOpened():
  ret, frame = cap.read()
  cv_bridge.cv2_to_imgmsg(frame)
  pub.publish(cv_bridge.cv2_to_imgmsg(frame))

rclpy.shutdown()
