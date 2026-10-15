"""
Puts the camera's classes on the lidar's tracks.

  in   tracked_markers     visualization_msgs/MarkerArray, from pcl_tracker
       detections          yolo_msgs/DetectionArray, in rectified pixels
       camera_info         sensor_msgs/CameraInfo of the image they came from
       image               sensor_msgs/Image, only read with debug_image
  out  classified_markers  tracked_markers, each box's text set to its class
       ~/debug_image       the image, with the tracks drawn where they project

The lidar says where things are and the camera says what they are. Each camera
frame, the lidar's confirmed tracks are projected into the image and paired
with detections by bearing; a pairing is a vote for that class on that track
ID, and a track takes a class once one is clearly ahead.

Every lidar frame goes straight back out, labelled or not, so a camera that
stops costs the labels and nothing else: the planner still sees every obstacle.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray

from prop_fusion.association import Votes, associate, corners, project, unoccluded


def is_detection(marker):
    return marker.type == Marker.CUBE and marker.action == Marker.ADD


def rotation_of(q):
    # pcl_tracker always sets w, but a zero quaternion would throw.
    if q.x == q.y == q.z == q.w == 0.0:
        return np.eye(3)
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


class Fusion(Node):
    def __init__(self):
        super().__init__("fusion")
        self.max_age = self.declare_parameter("max_age", 0.5).value
        self.bearing_tolerance = math.radians(
            self.declare_parameter("bearing_tolerance", 3.0).value,
        )
        self.votes = Votes(self.declare_parameter("vote_margin", 2.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tracks = []
        self.camera = None
        self.image = None

        self.pub = self.create_publisher(MarkerArray, "classified_markers", 10)
        self.create_subscription(MarkerArray, "tracked_markers", self.on_tracks, 10)
        self.create_subscription(
            DetectionArray,
            "detections",
            self.on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "camera_info",
            self.on_camera_info,
            qos_profile_sensor_data,
        )

        self.debug_pub = None
        if self.declare_parameter("debug_image", False).value:
            from cv_bridge import CvBridge

            self.bridge = CvBridge()
            self.debug_pub = self.create_publisher(Image, "~/debug_image", 1)
            self.create_subscription(
                Image,
                "image",
                lambda msg: setattr(self, "image", msg),
                qos_profile_sensor_data,
            )

    def on_camera_info(self, msg):
        P = np.array(msg.p, dtype=float).reshape(3, 4)
        if not P.any():
            P = np.c_[np.array(msg.k, dtype=float).reshape(3, 3), np.zeros(3)]
        self.camera = (P, msg.width, msg.height)

    def on_tracks(self, msg):
        self.tracks = [m for m in msg.markers if is_detection(m)]
        self.votes.keep({m.id for m in self.tracks})
        for marker in self.tracks:
            marker.text = self.votes.label(marker.id)
        self.pub.publish(msg)

    def on_detections(self, msg):
        if not self.tracks or self.camera is None:
            return
        source = self.tracks[0].header
        age = (
            abs(
                (
                    Time.from_msg(msg.header.stamp) - Time.from_msg(source.stamp)
                ).nanoseconds,
            )
            * 1e-9
        )
        if age > self.max_age:
            self.get_logger().warn(
                f"camera and lidar frames are {age:.2f} s apart, not comparing them",
                throttle_duration_sec=5.0,
            )
            return

        transform = self.lookup(msg.header.frame_id, source.frame_id, msg.header.stamp)
        if transform is None:
            return
        rotation = rotation_of(transform.rotation)
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z],
        )

        P, width, height = self.camera
        projected = []
        for marker in self.tracks:
            box = corners(
                [
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ],
                [marker.scale.x, marker.scale.y, marker.scale.z],
                rotation_of(marker.pose.orientation),
            )
            in_camera = box @ rotation.T + translation
            pixels = project(in_camera, P, width, height)
            if pixels is not None:
                projected.append((marker, pixels, in_camera[:, 2].mean()))

        spans = [(p[0], p[2]) for _, p, _ in projected]
        visible = [
            projected[i] for i in unoccluded(spans, [d for _, _, d in projected])
        ]

        detections = [
            (
                d.bbox.center.position.x - d.bbox.size.x / 2.0,
                d.bbox.center.position.x + d.bbox.size.x / 2.0,
            )
            for d in msg.detections
        ]
        pairs = associate(
            [(p[0], p[2]) for _, p, _ in visible],
            detections,
            P[0, 0] * math.tan(self.bearing_tolerance),
        )
        for i, j in pairs:
            detection = msg.detections[j]
            self.votes.add(visible[i][0].id, detection.class_name, detection.score)

        if self.debug_pub is not None:
            self.draw(visible, msg.detections, pairs)

    def lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                stamp,
                Duration(seconds=0.05),
            ).transform
        except TransformException:
            pass
        try:
            return self.tf_buffer.lookup_transform(target, source, Time()).transform
        except TransformException as ex:
            self.get_logger().warn(
                f"no transform {source} -> {target}, skipping this camera frame: {ex}",
                throttle_duration_sec=5.0,
            )
            return None

    def draw(self, visible, detections, pairs):
        import cv2

        if self.image is None:
            return
        frame = self.bridge.imgmsg_to_cv2(self.image, "bgr8")

        def corner(u, v):
            return int(round(u)), int(round(v))

        for d in detections:
            c, s = d.bbox.center.position, d.bbox.size
            cv2.rectangle(
                frame,
                corner(c.x - s.x / 2, c.y - s.y / 2),
                corner(c.x + s.x / 2, c.y + s.y / 2),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"{d.class_name} {d.score:.2f}",
                corner(c.x - s.x / 2, c.y + s.y / 2 + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        for marker, (left, top, right, bottom), _ in visible:
            cv2.rectangle(
                frame,
                corner(left, top),
                corner(right, bottom),
                (0, 220, 255),
                2,
            )
            cv2.putText(
                frame,
                f"#{marker.id} {self.votes.label(marker.id)}",
                corner(left, top - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 255),
                1,
            )
        for i, j in pairs:
            _, (left, top, right, bottom), _ = visible[i]
            c = detections[j].bbox.center.position
            cv2.line(
                frame,
                corner((left + right) / 2, (top + bottom) / 2),
                corner(c.x, c.y),
                (255, 0, 255),
                2,
            )

        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out.header = self.image.header
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(Fusion())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
