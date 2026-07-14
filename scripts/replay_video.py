import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rclpy.init()

cv_bridge = CvBridge()
node = Node("replay_video")

pub = node.create_publisher(Image, "/front_camera/image_raw", 10)

cap = cv2.VideoCapture(sys.argv[1])
while True:
    ret, frame = cap.read()
    if frame is None:
        break
    pub.publish(cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
    time.sleep(1 / 30)

rclpy.shutdown()
