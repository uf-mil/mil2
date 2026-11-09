#!/usr/bin/env python3
# generate detections on buoys from camera frames
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from detection import Detector
from light_detection import Light_Detector
from localization import Localizer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tracking import Tracker
from undistort import Undistorter
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        default_config = os.path.join(
            get_package_share_directory("object_classifier"),
            "config",
            "cam.yaml",
        )
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("model_path", os.path.expanduser("~/buoy_buoy10-13.pt"))
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("image_topic", "camera/img_raw")
        self.declare_parameter("light_padding_above", 0.25)
        self.declare_parameter("light_max_gap", 0.5)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            raise RuntimeError("model_path parameter is required")
        conf = self.get_parameter("conf").value

        self.undistorter = Undistorter(self.get_parameter("config_file").value)
        self.detector = Detector(model_path, conf=conf)
        self.tracker = Tracker(self.detector.model, conf=conf)
        self.localizer = Localizer()
        self.bridge = CvBridge()
        self.light_detector = Light_Detector(
            padding_above=self.get_parameter("light_padding_above").value,
            max_gap=self.get_parameter("light_max_gap").value,
        )

        self.detect_publisher = self.create_publisher(
            Detection2DArray,
            "camera/detections",
            10,
        )

        self.light_publisher = self.create_publisher(
            String,
            "camera/lights",
            10,
        )
        self.img_publisher = self.create_publisher(Image, "camera/detections/image", 10)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.on_img,
            qos,
        )
        self.get_logger().info(
            f"loaded {model_path}, classes: {self.detector.model.names}",
        )

    def on_img(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        corrected = self.undistorter.apply(frame)  # undistort
        tracks = self.tracker.track(corrected)  # detect and track

        for t in tracks:  # depth estimation
            t.update(self.localizer.locate(t, self.undistorter.intrinsics))

        self.detect_publisher.publish(self.to_msg(tracks, msg.header))

        # detect lights on buoys
        stamp = msg.header.stamp
        lights = self.light_detector.detect_light(
            corrected,
            tracks,
            stamp.sec + stamp.nanosec * 1e-9,
        )
        self.light_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
                        "frame_id": msg.header.frame_id,
                        "lights": lights,
                    },
                ),
            ),
        )

        annotated = self.bridge.cv2_to_imgmsg(
            self.detector.draw(corrected, tracks),
            encoding="bgr8",
        )
        annotated.header = msg.header

        # publish
        self.img_publisher.publish(annotated)

    @staticmethod
    def to_msg(tracks, header):
        # transform annotations dictionary into Detection2DArray
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
