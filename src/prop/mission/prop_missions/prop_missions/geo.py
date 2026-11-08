"""Latitude and longitude of points in the boat's local frame."""

import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

EARTH_RADIUS = 6378137.0  # m


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def offset_latlon(latitude, longitude, east, north):
    """The point east and north metres from a latitude and longitude."""
    lat = latitude + math.degrees(north / EARTH_RADIUS)
    lon = longitude + math.degrees(
        east / (EARTH_RADIUS * math.cos(math.radians(latitude))),
    )
    return lat, lon


class GeoReference:
    """
    Turns local (x, y) into latitude and longitude, from the last GPS fix and
    where the boat was in the local frame when it came.

    The local frame need not point east: ekf_local's yaw starts at zero wherever
    the boat faced. The IMU's yaw is east-referenced, so the difference between
    the two is how far the local frame is turned from east.
    """

    def __init__(self, node, gps_topic="gps/fix", imu_topic="imu/data"):
        self.fix = None  # (latitude, longitude, x, y)
        self.pose = None  # (x, y, yaw) from odometry
        self.turn = None  # local frame's x axis, radians from east
        node.create_subscription(
            NavSatFix,
            gps_topic,
            self.on_fix,
            qos_profile_sensor_data,
        )
        node.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)

    def on_odometry(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def on_fix(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and self.pose is not None:
            self.fix = (msg.latitude, msg.longitude, self.pose[0], self.pose[1])

    def on_imu(self, msg):
        if self.pose is not None:
            turn = yaw_of(msg.orientation) - self.pose[2]
            self.turn = math.atan2(math.sin(turn), math.cos(turn))

    def ready(self):
        return self.fix is not None and self.turn is not None

    def latlon(self, x, y):
        """(latitude, longitude) of a local point, or None before a fix."""
        if not self.ready():
            return None
        latitude, longitude, fx, fy = self.fix
        dx, dy = x - fx, y - fy
        c, s = math.cos(self.turn), math.sin(self.turn)
        return offset_latlon(latitude, longitude, c * dx - s * dy, s * dx + c * dy)

    def local(self, latitude, longitude):
        """(x, y) of a latitude and longitude, or None before a fix."""
        if not self.ready():
            return None
        lat0, lon0, fx, fy = self.fix
        north = math.radians(latitude - lat0) * EARTH_RADIUS
        east = (
            math.radians(longitude - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
        )
        c, s = math.cos(self.turn), math.sin(self.turn)
        return fx + c * east + s * north, fy - s * east + c * north
