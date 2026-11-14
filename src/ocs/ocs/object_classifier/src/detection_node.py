#!/usr/bin/env python3
# generate detections on buoys from camera frames

# Undistort camera frames for yolo_ros, then attach range estimates to its detections.
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from light_detection import Light_Detector
from localization import Localizer
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from undistort import Undistorter
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from yolo_msgs.msg import DetectionArray


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        default_config = os.path.join(
            get_package_share_directory("object_classifier"),
            "config",
            "cam.yaml",
        )
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("image_topic", "cam/image/raw")
        self.declare_parameter("undistorted_image_topic", "cam/image/undistorted")
        self.declare_parameter("yolo_detections_topic", "yolo/tracking")

        self.undistorter = Undistorter(self.get_parameter("config_file").value)
        self.localizer = Localizer()
        self.bridge = CvBridge()
        self.light_detector = Light_Detector(
            padding_above=self.get_parameter("light_padding_above").value,
            max_gap=self.get_parameter("light_max_gap").value,
        )

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.undistorted_pub = self.create_publisher(
            Image,
            self.get_parameter("undistorted_image_topic").value,
            qos,
        )
        self.detect_publisher = self.create_publisher(
            Detection2DArray,
            "camera/detections",
            10,
        )

        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.on_img,
            qos,
        )
        self.create_subscription(
            DetectionArray,
            self.get_parameter("yolo_detections_topic").value,
            self.on_yolo_detections,
            10,
        )
        self.get_logger().info(
            "undistorting "
            f"{self.get_parameter('image_topic').value} -> "
            f"{self.get_parameter('undistorted_image_topic').value}; "
            "localizing "
            f"{self.get_parameter('yolo_detections_topic').value}",
        )

    def on_img(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corrected = self.undistorter.apply(frame)
        out = self.bridge.cv2_to_imgmsg(corrected, encoding="bgr8")
        out.header = msg.header
        self.undistorted_pub.publish(out)

    def on_yolo_detections(self, msg):
        if self.undistorter.intrinsics is None:
            return

        tracks = []
        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size.x
            h = det.bbox.size.y
            track = {
                "box": [cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0],
                "label": det.class_name,
                "conf": det.score,
                "track_id": det.id or det.class_name,
            }
            track.update(self.localizer.locate(track, self.undistorter.intrinsics))
            tracks.append(track)

        self.detect_publisher.publish(self.to_msg(tracks, msg.header))

    @staticmethod
    def to_msg(tracks, header):
        out = Detection2DArray(header=header)
        for t in tracks:
            x1, y1, x2, y2 = t["box"]
            det = Detection2D(header=header, id=str(t["track_id"]))
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = t["label"]
            hyp.hypothesis.score = t["conf"]
            if t["range_m"] is not None:
                hyp.pose.pose.position.x = t["x_m"]
                hyp.pose.pose.position.y = t["y_m"]
                hyp.pose.pose.position.z = t["z_m"]
            det.results.append(hyp)
            out.detections.append(det)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
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
