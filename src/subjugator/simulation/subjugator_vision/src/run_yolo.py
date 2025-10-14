#!/usr/bin/env python3
import os

import rclpy
from cv_bridge import CvBridge
from mil_msgs.msg import PerceptionTarget, PerceptionTargetArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # publishers
        self.percep_pub = self.create_publisher(
            PerceptionTargetArray,
            "/perception/targets",
            10,
        )

        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("model_name", "new_sim_nav_channel.pt")
        image_topic = self.get_parameter("image_topic").value
        model_name = self.get_parameter("model_name").value

        self.bridge = CvBridge()
        self.create_subscription(Image, image_topic, self.cb, 10)
        self.pub_img = self.create_publisher(Image, "/yolo/image_annotated", 10)
        self.pub_det = self.create_publisher(Detection2DArray, "/yolo/detections", 10)
        self.get_logger().info(f"Subscribed to {image_topic}")

        pkg_dir = os.path.dirname(__file__)
        model_pat = os.path.join(pkg_dir, "models", model_name)
        self.model = YOLO(model_pat)
        self.model.fuse()
        self.get_logger().info("YOLO-v11 model ready")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(frame, conf=0.20, verbose=False)[0]

        arr = PerceptionTargetArray()
        arr.stamp = msg.header.stamp

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf)
            cls = int(box.cls)
            label = self.model.names[cls]

            # Detection2D
            det = Detection2D()
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label
            hypo.hypothesis.score = conf
            det.results.append(hypo)
            det_arr.detections.append(det)

            # PerceptionTarget
            pt = PerceptionTarget()
            pt.label = label
            pt.cx = det.bbox.center.position.x
            pt.cy = det.bbox.center.position.y
            pt.width = det.bbox.size_x
            pt.height = det.bbox.size_y
            pt.confidence = conf
            pt.stamp = msg.header.stamp
            arr.targets.append(pt)

        # Overlay image
        annotated = results.plot()
        ros_img = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        ros_img.header = msg.header

        # publish everything once per frame
        self.percep_pub.publish(arr)
        self.pub_det.publish(det_arr)
        self.pub_img.publish(ros_img)


def main():
    rclpy.init()
    rclpy.spin(YoloDetector())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
