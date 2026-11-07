"""
Keeps the cameras' classes on the evidence grid, and publishes what that labels.

  in   occupancy_grid      nav_msgs/OccupancyGrid, 0..100 evidence per cell
       classified_markers  visualization_msgs/MarkerArray, from fusion
  out  labelled_objects    mil_msgs/PerceptionObjectArray, in the grid's frame
       labelled_markers    visualization_msgs/MarkerArray, the same objects for RViz

An object's class is in labeled_classification, its pose is the centre of its
cells and its scale is their extent. Its id stays with it for as long as the
grid remembers its cells, and is never reused.
"""

import numpy as np
from geometry_msgs.msg import PointStamped
from mil_msgs.msg import PerceptionObject, PerceptionObjectArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from prop_fusion.object_map import ObjectMap, Window
from prop_fusion.run import run_until_stopped

OBJECT_COLOUR = ColorRGBA(r=1.0, g=0.1, b=0.8, a=0.8)
TEXT_COLOUR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
OBJECT_HEIGHT = 1.0  # m, only how tall it draws


def perception_object(found, header):
    msg = PerceptionObject()
    msg.header = header
    msg.id = found.id
    msg.labeled_classification = found.label
    msg.pose.position.x = found.x
    msg.pose.position.y = found.y
    msg.scale.x = found.size_x
    msg.scale.y = found.size_y
    return msg


def object_markers(objects, header):
    """Each object as a column over its cells, with its name floating above."""
    markers = [Marker(header=header, action=Marker.DELETEALL)]
    for found in objects:
        column = Marker(
            header=header,
            ns="objects",
            id=found.id,
            type=Marker.CYLINDER,
            color=OBJECT_COLOUR,
        )
        column.pose.position.x = found.x
        column.pose.position.y = found.y
        column.pose.position.z = OBJECT_HEIGHT / 2
        column.scale.x = found.size_x
        column.scale.y = found.size_y
        column.scale.z = OBJECT_HEIGHT

        name = Marker(
            header=header,
            ns="names",
            id=found.id,
            type=Marker.TEXT_VIEW_FACING,
            color=TEXT_COLOUR,
            text=f"{found.label} #{found.id}",
        )
        name.pose.position.x = found.x
        name.pose.position.y = found.y
        name.pose.position.z = OBJECT_HEIGHT + 0.5
        name.scale.z = 0.6
        markers += [column, name]
    return MarkerArray(markers=markers)


class ObjectMapNode(Node):
    def __init__(self):
        super().__init__("object_map")
        self.map = ObjectMap(
            occupied=self.declare_parameter("occupied", 60).value,
            label_radius=self.declare_parameter("label_radius", 0.6).value,
            max_cluster_size=self.declare_parameter("max_cluster_size", 2.5).value,
        )
        self.grid_frame = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PerceptionObjectArray, "labelled_objects", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "labelled_markers", 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.on_grid, 1)
        self.create_subscription(
            MarkerArray,
            "classified_markers",
            self.on_classified,
            10,
        )

    def on_grid(self, msg):
        self.grid_frame = msg.header.frame_id
        info = msg.info
        evidence = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        corner = info.origin.position
        self.map.update_grid(evidence, Window(corner.x, corner.y, info.resolution))

        found = self.map.objects()
        objects = [perception_object(o, msg.header) for o in found]
        self.pub.publish(PerceptionObjectArray(objects=objects))
        self.marker_pub.publish(object_markers(found, msg.header))

    def on_classified(self, msg):
        if self.grid_frame is None:
            return
        for marker in msg.markers:
            if marker.text:
                position = self.in_grid_frame(marker)
                if position is not None:
                    self.map.add_label(marker.text, position.x, position.y)

    def in_grid_frame(self, marker):
        """Where a marker sits in the grid's frame, or None if TF cannot say."""
        frame = marker.header.frame_id
        if frame == self.grid_frame:
            return marker.pose.position
        try:
            transform = self.tf_buffer.lookup_transform(self.grid_frame, frame, Time())
        except TransformException as ex:
            self.get_logger().warn(
                f"no transform {frame} -> {self.grid_frame}, skipping a label: {ex}",
                throttle_duration_sec=5.0,
            )
            return None
        point = PointStamped(point=marker.pose.position)
        return do_transform_point(point, transform).point


def main():
    run_until_stopped(ObjectMapNode)


if __name__ == "__main__":
    main()
