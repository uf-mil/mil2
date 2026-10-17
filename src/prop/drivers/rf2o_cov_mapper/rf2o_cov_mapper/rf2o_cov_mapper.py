"""Republish rf2o odometry with a covariance chosen from how many beams its scans had.

rf2o publishes all-zero covariance, which robot_localization treats as near
perfect, and its internal estimate is far too small and ignores sparse scans.
It also publishes vy = 0 though its pose includes sideways motion.

Every rf2o reading, relayed or dropped, gets a report on trust_topic: one topic
to record when checking the relay on the boat.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

# Indices of the diagonal in a 6x6 row-major covariance array.
DIAGONAL = (0, 7, 14, 21, 28, 35)

# Large value for unobserved dimensions, per robot_localization convention.
UNOBSERVED = 1e6

SCANS_KEPT = 10


def beam_count(scan: LaserScan) -> int:
    return sum(
        1
        for r in scan.ranges
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max
    )


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def sideways_speed(previous: Odometry, current: Odometry, dt: float) -> float:
    """Sideways speed [m/s], measured across the mid-step heading."""
    dx = current.pose.pose.position.x - previous.pose.pose.position.x
    dy = current.pose.pose.position.y - previous.pose.pose.position.y
    turn = yaw_of(current) - yaw_of(previous)
    yaw = yaw_of(previous) + math.atan2(math.sin(turn), math.cos(turn)) / 2
    return (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt


def stamp_ns(msg) -> int:
    return Time.from_msg(msg.header.stamp).nanoseconds


def verdict(dt, beams, min_beams, full_trust_beams, max_scan_gap) -> str:
    """Why a reading is dropped, or which trust tier it gets.

    beams holds the counts of the two scans rf2o matched; None if one was not seen.
    """
    # When a scan match fails, rf2o still publishes: the previous estimate, with its stamp.
    if dt <= 0.0:
        return "dropped: repeated stamp"
    if dt > max_scan_gap:
        return f"dropped: {dt:.1f} s since the last reading"
    if None in beams:
        return "dropped: no matching scan"
    # rf2o's estimate comes from matching two scans, so it is only as good as the sparser one.
    if min(beams) < min_beams:
        return "dropped: too few beams"
    return "full trust" if min(beams) >= full_trust_beams else "low trust"


class Rf2oCovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__("rf2o_covariance_relay")

        self.declare_parameter("input_topic", "/odom_rf2o")
        self.declare_parameter("output_topic", "/lidar/odom")
        self.declare_parameter("scan_topic", "/lidar/above_water_scan")
        self.declare_parameter("trust_topic", "/lidar/odom_trust")

        # Order: x, y, z, roll, pitch, yaw
        self.declare_parameter(
            "pose_variances",
            [0.001, 0.001, UNOBSERVED, UNOBSERVED, UNOBSERVED, 0.003],
        )
        # Order: vx, vy, vz, vroll, vpitch, vyaw
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
        # With nothing above the water the scan stops, and rf2o's next match spans the gap.
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
        self._trust_pub = self.create_publisher(
            DiagnosticStatus,
            self.get_parameter("trust_topic").value,
            10,
        )
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
        beams = [self._beams_by_stamp.get(stamp_ns(m)) for m in (previous, msg)]
        result = verdict(
            dt,
            beams,
            self._min_beams,
            self._full_trust_beams,
            self._max_scan_gap,
        )
        if result.startswith("dropped"):
            if result == "dropped: no matching scan":
                self.get_logger().warning(
                    "no scan matches this odometry stamp, dropping it",
                    throttle_duration_sec=5.0,
                )
            self._report(msg, beams, result, sd=None)
            return

        msg.twist.twist.linear.y = sideways_speed(previous, msg, dt)
        twist_var = (
            self._twist_var if result == "full trust" else self._sparse_twist_var
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
        self._report(msg, beams, result, sd=math.sqrt(twist_var[0]))

    def _report(self, msg: Odometry, beams, result: str, sd: float | None) -> None:
        values = {
            "stamp": f"{stamp_ns(msg) * 1e-9:.3f}",
            "beams_previous": str(beams[0]),
            "beams_current": str(beams[1]),
            "vx": f"{msg.twist.twist.linear.x:.3f}",
            "vy": f"{msg.twist.twist.linear.y:.3f}",
            "wz": f"{msg.twist.twist.angular.z:.3f}",
            "speed_sd": "none" if sd is None else f"{sd:.3f}",
        }
        self._trust_pub.publish(
            DiagnosticStatus(
                level=(
                    DiagnosticStatus.OK
                    if result == "full trust"
                    else DiagnosticStatus.WARN
                ),
                name="rf2o relay",
                message=result,
                values=[KeyValue(key=k, value=v) for k, v in values.items()],
            ),
        )


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
