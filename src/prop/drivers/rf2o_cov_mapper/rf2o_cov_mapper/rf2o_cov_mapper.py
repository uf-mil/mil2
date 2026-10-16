"""Republish rf2o laser odometry with a covariance set by how much the scan can see.

rf2o_laser_odometry leaves pose.covariance and twist.covariance filled with
zeros. robot_localization does not reject a zero covariance -- it clamps the
diagonal to a tiny epsilon, which reads as an almost perfect measurement.

rf2o's own least-squares covariance is no substitute: it is far smaller than
rf2o's real error and does not grow as the scan empties. The error does track
how many beams returned, so the variance here is chosen from that count,
and a scan with almost nothing in it is not relayed at all.

rf2o also publishes zero sideways speed, though its pose integrates sideways
motion; the relay recovers vy from consecutive poses.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

# Indices of the diagonal in a 6x6 row-major covariance array.
DIAGONAL = (0, 7, 14, 21, 28, 35)

# Large value for unobserved dimensions, per robot_localization convention.
UNOBSERVED = 1e6

# rf2o stamps its odometry with the scan it used; a few scans of slack covers
# odometry arriving after the next scan.
SCANS_KEPT = 10


def beam_count(scan: LaserScan) -> int:
    """Number of beams with a return inside the scan's valid range."""
    return sum(
        1
        for r in scan.ranges
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max
    )


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def sideways_speed(previous: Odometry, current: Odometry, dt: float) -> float:
    """Sideways speed [m/s] between two poses, across the mid-step heading."""
    dx = current.pose.pose.position.x - previous.pose.pose.position.x
    dy = current.pose.pose.position.y - previous.pose.pose.position.y
    turn = yaw_of(current) - yaw_of(previous)
    yaw = yaw_of(previous) + math.atan2(math.sin(turn), math.cos(turn)) / 2
    return (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt


def stamp_ns(msg) -> int:
    return Time.from_msg(msg.header.stamp).nanoseconds


class Rf2oCovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__("rf2o_covariance_relay")

        self.declare_parameter("input_topic", "/odom_rf2o")
        self.declare_parameter("output_topic", "/lidar/odom")
        self.declare_parameter("scan_topic", "/lidar/above_water_scan")

        # Order: x, y, z, roll, pitch, yaw
        self.declare_parameter(
            "pose_variances",
            [0.001, 0.001, UNOBSERVED, UNOBSERVED, UNOBSERVED, 0.003],
        )
        # Order: vx, vy, vz, vroll, vpitch, vyaw. The first applies from
        # full_trust_beams up, the sparse one from min_beams up.
        self.declare_parameter(
            "twist_variances",
            [0.0064, 0.0064, UNOBSERVED, UNOBSERVED, UNOBSERVED, 0.005],
        )
        self.declare_parameter(
            "sparse_twist_variances",
            [0.0625, 0.0625, UNOBSERVED, UNOBSERVED, UNOBSERVED, 0.005],
        )
        self.declare_parameter("full_trust_beams", 50)
        self.declare_parameter("min_beams", 10)
        # The scan stops entirely when nothing is above the water, and rf2o then
        # matches the next scan against one from however long ago.
        self.declare_parameter("max_scan_gap", 0.5)

        # Optional: rewrite frame ids so rf2o's own "odom" frame does not
        # collide with the frame the EKF owns. Leave empty to pass through.
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "")

        self._pose_var = list(self.get_parameter("pose_variances").value)
        self._twist_var = list(self.get_parameter("twist_variances").value)
        self._sparse_twist_var = list(
            self.get_parameter("sparse_twist_variances").value,
        )
        self._full_trust_beams = self.get_parameter("full_trust_beams").value
        self._min_beams = self.get_parameter("min_beams").value
        self._max_scan_gap = self.get_parameter("max_scan_gap").value
        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value

        self._beams_by_stamp: dict[int, int] = {}
        self._previous: Odometry | None = None

        out_topic = self.get_parameter("output_topic").value
        in_topic = self.get_parameter("input_topic").value
        scan_topic = self.get_parameter("scan_topic").value

        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, in_topic, self._on_odom, 10)

        self.get_logger().info(
            f"relaying {in_topic} -> {out_topic}, trust from beams on {scan_topic}",
        )

    def _on_scan(self, scan: LaserScan) -> None:
        self._beams_by_stamp[stamp_ns(scan)] = beam_count(scan)
        while len(self._beams_by_stamp) > SCANS_KEPT:
            del self._beams_by_stamp[next(iter(self._beams_by_stamp))]

    def _on_odom(self, msg: Odometry) -> None:
        previous, self._previous = self._previous, msg
        if previous is None:
            return
        dt = (stamp_ns(msg) - stamp_ns(previous)) * 1e-9
        # rf2o republishes its last estimate, old stamp and all, when no new scan was matched.
        if not 0.0 < dt <= self._max_scan_gap:
            return
        # rf2o's estimate comes from matching this scan against the previous one,
        # so it is only as good as the sparser of the two.
        pair = [self._beams_by_stamp.get(stamp_ns(m)) for m in (previous, msg)]
        if None in pair:
            self.get_logger().warning(
                "no scan matches this odometry stamp, dropping it",
                throttle_duration_sec=5.0,
            )
            return
        beams = min(pair)
        if beams < self._min_beams:
            return

        msg.twist.twist.linear.y = sideways_speed(previous, msg, dt)
        twist_var = (
            self._twist_var
            if beams >= self._full_trust_beams
            else self._sparse_twist_var
        )
        for slot, variance in zip(DIAGONAL, self._pose_var):
            msg.pose.covariance[slot] = variance
        for slot, variance in zip(DIAGONAL, twist_var):
            msg.twist.covariance[slot] = variance

        if self._frame_id:
            msg.header.frame_id = self._frame_id
        if self._child_frame_id:
            msg.child_frame_id = self._child_frame_id

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Rf2oCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
