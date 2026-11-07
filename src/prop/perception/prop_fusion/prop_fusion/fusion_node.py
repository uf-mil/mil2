"""
Labels the lidar's tracks with what the cameras see.

  in   tracked_markers        visualization_msgs/MarkerArray, from pcl_tracker
       /<camera>/detections   yolo_msgs/DetectionArray, in rectified pixels
       /<camera>/camera_info  sensor_msgs/CameraInfo of that rectified image
       /<camera>/image_rect   sensor_msgs/Image, only read with debug_image
  out  classified_markers     tracked_markers, each box's text set to its class
       ~/<camera>/debug_image tracks and detections drawn on the image

The cameras are the ones named in sensors.yaml. The lidar says where things
are and the cameras say what they are: each camera frame, the confirmed tracks
are projected into the image and paired with detections by bearing, and each
pair is a vote for that class on that track.

Every lidar frame goes straight back out, labelled or not, so a camera that
stops costs the labels and nothing else.
"""

import math

import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray

from prop_fusion.association import Votes, associate
from prop_fusion.mounts import Sensors, default_sensors_file
from prop_fusion.overlay import draw
from prop_fusion.projection import Lens, corners, unoccluded
from prop_fusion.run import run_until_stopped


def is_track(marker):
    return marker.type == Marker.CUBE and marker.action == Marker.ADD


def rotation_of(q):
    if q.x == q.y == q.z == q.w == 0.0:
        return np.eye(3)
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def lens_of(info):
    return Lens(np.array(info.p, dtype=float).reshape(3, 4), info.width, info.height)


def box_of(detection):
    c, s = detection.bbox.center.position, detection.bbox.size
    return c.x - s.x / 2, c.y - s.y / 2, c.x + s.x / 2, c.y + s.y / 2


def seconds_between(a, b):
    return abs((Time.from_msg(a) - Time.from_msg(b)).nanoseconds) * 1e-9


class CameraView:
    """One camera's topics, and where the lidar's tracks land in its image."""

    def __init__(self, node, name, on_detections, debug):
        self.name = name
        self.lens = None
        self.image = None
        self.debug_pub = None

        node.create_subscription(
            CameraInfo,
            f"/{name}/camera_info",
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            DetectionArray,
            f"/{name}/detections",
            lambda msg: on_detections(self, msg),
            qos_profile_sensor_data,
        )
        if debug:
            self.bridge = CvBridge()
            self.debug_pub = node.create_publisher(Image, f"~/{name}/debug_image", 1)
            node.create_subscription(
                Image,
                f"/{name}/image_rect",
                self.on_image,
                qos_profile_sensor_data,
            )

    def on_camera_info(self, msg):
        self.lens = lens_of(msg)

    def on_image(self, msg):
        self.image = msg

    def visible(self, tracks, rotation, translation):
        """(marker, pixel box) for each track in view and not hidden behind another."""
        seen = []
        for marker in tracks:
            p, s = marker.pose.position, marker.scale
            box = corners(
                [p.x, p.y, p.z],
                [s.x, s.y, s.z],
                rotation_of(marker.pose.orientation),
            )
            in_camera = box @ rotation.T + translation
            pixels = self.lens.box(in_camera)
            if pixels is not None:
                seen.append((marker, pixels, in_camera[:, 2].mean()))
        spans = [(px[0], px[2]) for _, px, _ in seen]
        keep = unoccluded(spans, [depth for *_, depth in seen])
        # return marker and pixel box
        return [seen[i][:2] for i in keep]

    def publish_debug(self, visible, detections, pairs, label):
        if self.debug_pub is None or self.image is None:
            return
        frame = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        draw(
            frame,
            [(box_of(d), f"{d.class_name} {d.score:.2f}") for d in detections],
            [(px, f"#{m.id} {label(m.id)}") for m, px in visible],
            pairs,
        )
        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out.header = self.image.header
        self.debug_pub.publish(out)


class Fusion(Node):
    def __init__(self):
        super().__init__("fusion")
        self.max_age = self.declare_parameter("max_age", 0.5).value
        self.bearing_tolerance = math.radians(
            self.declare_parameter("bearing_tolerance", 3.0).value,
        )
        self.votes = Votes(self.declare_parameter("vote_margin", 2.0).value)
        debug = self.declare_parameter("debug_image", False).value
        sensors_file = self.declare_parameter(
            "sensors_file",
            default_sensors_file(),
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tracks = []
        self.tracks_header = None

        self.pub = self.create_publisher(MarkerArray, "classified_markers", 10)
        self.create_subscription(MarkerArray, "tracked_markers", self.on_tracks, 10)

        cameras = Sensors.load(sensors_file).cameras
        self.views = [
            CameraView(self, name, self.on_detections, debug) for name in cameras
        ]
        self.get_logger().info(
            f"fusing the lidar with {', '.join(cameras) or 'no cameras'}",
        )

    def on_tracks(self, msg):
        self.tracks = [m for m in msg.markers if is_track(m)]
        self.tracks_header = self.tracks[0].header if self.tracks else None
        self.votes.keep({m.id for m in self.tracks})
        for marker in self.tracks:
            marker.text = self.votes.label(marker.id)
        self.pub.publish(msg)

    def on_detections(self, view, msg):
        if self.tracks_header is None or view.lens is None:
            return
        age = seconds_between(msg.header.stamp, self.tracks_header.stamp)
        if age > self.max_age:
            self.get_logger().warn(
                f"{view.name} and lidar frames are {age:.2f} s apart, not comparing them",
                throttle_duration_sec=5.0,
            )
            return

        transform = self.lookup(
            msg.header.frame_id,
            self.tracks_header.frame_id,
            msg.header.stamp,
        )
        if transform is None:
            return
        rotation, translation = transform

        visible = view.visible(self.tracks, rotation, translation)
        pairs = associate(
            [(px[0], px[2]) for _, px in visible],
            [(box[0], box[2]) for box in map(box_of, msg.detections)],
            view.lens.fx * math.tan(self.bearing_tolerance),
        )
        for i, j in pairs:
            detection = msg.detections[j]
            self.votes.add(visible[i][0].id, detection.class_name, detection.score)

        view.publish_debug(visible, msg.detections, pairs, self.votes.label)

    def lookup(self, target, source, stamp):
        """Rotation matrix and translation taking source points into target."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source, stamp)
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time())
            except TransformException as ex:
                self.get_logger().warn(
                    f"no transform {source} -> {target}, skipping this frame: {ex}",
                    throttle_duration_sec=5.0,
                )
                return None
        t = tf.transform.translation
        return rotation_of(tf.transform.rotation), np.array([t.x, t.y, t.z])


def main():
    run_until_stopped(Fusion)


if __name__ == "__main__":
    main()
