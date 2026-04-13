import math

from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.marker_pb2 import Marker
from gz.transport13 import Node as GZNode

from mil_robogym.data_collection.types import Coord4D


class DrawMarkerClient:
    """
    Client that draws cone markers in gz sim.
    """

    def __init__(self):

        self.gz_node = GZNode()

        self.marker_counter = 0

        self.timeout = 100

    def clear_markers(self):
        """
        Remove all markers from the scene.
        """

        request = Marker()
        request.action = Marker.DELETE_ALL

        success, response = self.gz_node.request(
            "/marker",
            request,
            Marker,
            Empty,
            self.timeout,
        )

    def place_marker(self, coordinate: Coord4D, main: bool = True):
        """
        Place marker.
        """
        request = Marker()

        request.action = Marker.ADD_MODIFY
        request.type = Marker.CONE
        request.ns = "default"
        request.id = self.marker_counter

        if main:
            request.scale.x = 0.3
            request.scale.y = 0.3
            request.scale.z = 0.5
        else:
            request.scale.x = 0.1
            request.scale.y = 0.1
            request.scale.z = 0.15

        x, y, z, yaw = coordinate
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z

        x, y, z, w = self._get_quaternion_from_yaw(yaw)
        request.pose.orientation.x = x
        request.pose.orientation.y = y
        request.pose.orientation.z = z
        request.pose.orientation.w = w

        request.material.diffuse.r = 1.0
        request.material.diffuse.g = 0.0
        request.material.diffuse.b = 0.0
        request.material.diffuse.a = 1.0

        request.material.ambient.r = 1.0
        request.material.ambient.g = 0.0
        request.material.ambient.b = 0.0
        request.material.ambient.a = 1.0

        self.gz_node.request(
            "/marker",
            request,
            Marker,
            Empty,
            self.timeout,
        )

        self.marker_counter += 1

    def slerp(self, coord1: Coord4D, coord2: Coord4D, num_points: int = 5):
        """
        Interpolate between two Coord4D points and place markers.

        coord1: (x, y, z, yaw)
        coord2: (x, y, z, yaw)
        num_points: number of intermediate points (excluding endpoints)
        """

        x1, y1, z1, yaw1 = coord1
        x2, y2, z2, yaw2 = coord2

        # Normalize yaw difference to [-pi, pi]
        def shortest_angle_diff(a, b):
            diff = b - a
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        yaw_diff = shortest_angle_diff(yaw1, yaw2)

        for i in range(1, num_points + 1):
            t = i / (num_points + 1)

            # Linear interpolation for position
            x = (1 - t) * x1 + t * x2
            y = (1 - t) * y1 + t * y2
            z = (1 - t) * z1 + t * z2

            # "SLERP" for yaw (shortest path interpolation)
            yaw = yaw1 + t * yaw_diff

            self.place_marker((x, y, z, yaw), main=False)

    def _get_quaternion_from_yaw(self, yaw: float) -> tuple[float, float, float, float]:
        """
        Returns quaternion for:
        yaw about Z + 90 deg rotation about X
        """

        # Yaw quaternion (Z axis)
        sz = math.sin(yaw / 2.0)
        cz = math.cos(yaw / 2.0)
        q_yaw = (0.0, 0.0, sz, cz)

        # 90 deg rotation about X
        angle = math.pi / 2.0
        sx = math.sin(angle / 2.0)
        cx = math.cos(angle / 2.0)
        q_x = (sx, 0.0, 0.0, cx)

        # Quaternion multiplication: q = q_yaw * q_x
        x1, y1, z1, w1 = q_yaw
        x2, y2, z2, w2 = q_x

        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        return (x, y, z, w)
