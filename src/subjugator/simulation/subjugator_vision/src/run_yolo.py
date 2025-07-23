#!/usr/bin/env python3

import os
import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
from geometry_msgs.msg import Point
from vision_msgs.msg import Pose2D, Point2D
from mil_msgs.msg import PerceptionTarget


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.percep_pub = self.create_publisher(
                    PerceptionTarget, "/perception/targets", 10)
        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("model_name",  "project_38.pt")
        image_topic = self.get_parameter("image_topic").value
        model_name  = self.get_parameter("model_name").value

        self.bridge = CvBridge()
        self.create_subscription(Image, image_topic, self.cb, 10)
        self.pub_img = self.create_publisher(Image, "/yolo/image_annotated", 10)
        self.pub_det = self.create_publisher(Detection2DArray, "/yolo/detections", 10)

        self.get_logger().info(f"Subscribed to {image_topic}")

        # Get Model
        pkg_dir = os.path.dirname(__file__)
        model_pat = os.path.join(pkg_dir, "models", model_name)
        self.model = YOLO(model_pat)
        self.model.fuse()
        self.get_logger().info("YOLO-v11 model ready")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Ultralytics ‘predict’ returns a list of results (one per image);
        # streaming=False because we pass a single frame
        results = self.model.predict(frame, verbose=False)[0]
        self.get_logger().info(f"Found {len(results.boxes)} boxes")

        # Make Boxes
        annotated = results.plot()
        det_arr = Detection2DArray()
        det_arr.header = msg.header

        for box in results.boxes:

            # Get four corners and turn numbers into integers to for OpenCV and ROS
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf)
            cls  = int(box.cls)
            label = self.model.names[cls]

            det = Detection2D()
            center = Pose2D()
            center.position = Point2D(
                x=(x1 + x2) / 2.0,
                y=(y1 + y2) / 2.0,
                z=0.0
            )
            center.theta = 0.0 

            det.bbox.center = center

            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label
            hypo.hypothesis.score = conf
            det.results.append(hypo)
            det_arr.detections.append(det)

            
            pt = PerceptionTarget()
            pt.label = label 
            pt.cx = (x1 + x2) / 2.0
            pt.cy = (y1 + y2) / 2.0
            pt.width = float(x2 - x1)
            pt.height = float(y2 - y1)
            pt.confidence = conf
            pt.stamp = msg.header.stamp
            self.percep_pub.publish(pt)


        ros_img = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        ros_img.header = msg.header
        self.pub_img.publish(ros_img)
        self.pub_det.publish(det_arr)

def main():
    rclpy.init()
    rclpy.spin(YoloDetector())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
