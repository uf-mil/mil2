"""Republish rf2o odometry with a covariance chosen from how many beams its scans had.

rf2o publishes all-zero covariance, which robot_localization treats as near
perfect, and its internal estimate is far too small and ignores sparse scans.
It also publishes vy = 0 though its pose includes sideways motion.

rf2o goes with whatever most beams agree on, so a big moving object near us
(a kayak, another boat) can drag it. When enough small objects are in view
they vote on our velocity and rf2o is dropped if it disagrees with them; when
they are not, a scan that is mostly one object gets low trust.

Every rf2o reading, relayed or dropped, gets a report on trust_topic: one topic
to record when checking the relay on the boat.
"""

import math
from collections import deque
from dataclasses import dataclass
from statistics import median

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rf2o_cov_mapper.object_vote import GyroHeading, ObjectVote, Vote, rotate

# Indices of the diagonal in a 6x6 row-major covariance array.
DIAGONAL = (0, 7, 14, 21, 28, 35)

# Large value for unobserved dimensions, per robot_localization convention.
UNOBSERVED = 1e6

SCANS_KEPT = 10


def in_range(scan: LaserScan, r: float) -> bool:
    return math.isfinite(r) and scan.range_min <= r <= scan.range_max


def beam_count(scan: LaserScan) -> int:
    return sum(1 for r in scan.ranges if in_range(scan, r))


def largest_clump_share(scan: LaserScan, max_jump: float) -> float:
    """Share of the scan's beams in its largest run of neighbouring beams; 0 if it has none.

    A run breaks at a missing beam or where the range jumps by more than max_jump [m].
    """
    runs = []
    length = 0
    previous = None
    for r in scan.ranges:
        if not in_range(scan, r):
            r = None
        if length and (r is None or abs(r - previous) > max_jump):
            runs.append(length)
            length = 0
        if r is not None:
            length += 1
        previous = r
    if length:
        runs.append(length)
    if not runs:
        return 0.0

    first, last = scan.ranges[0], scan.ranges[-1]
    covered = scan.angle_max - scan.angle_min + scan.angle_increment
    if (
        len(runs) > 1
        and covered >= 2 * math.pi - 1e-3
        and in_range(scan, first)
        and in_range(scan, last)
        and abs(first - last) <= max_jump
    ):
        runs[0] += runs.pop()
    return max(runs) / sum(runs)


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def sideways_speed(previous: Odometry, current: Odometry, dt: float) -> float:
    """Sideways speed [m/s], measured across the mid-step heading."""
    dx = current.pose.pose.position.x - previous.pose.pose.position.x
    dy = current.pose.pose.position.y - previous.pose.pose.position.y
    turn = yaw_of(current) - yaw_of(previous)
    yaw = yaw_of(previous) + math.atan2(math.sin(turn), math.cos(turn)) / 2
    return rotate(dx, dy, -yaw)[1] / dt


def stamp_ns(msg) -> int:
    return Time.from_msg(msg.header.stamp).nanoseconds


@dataclass
class Referee:
    """The object vote, and how far rf2o's median velocity over its window is from it [m/s]."""

    vote: Vote
    gap: float


def referee_from(vote: Vote, readings) -> Referee | None:
    """Compare rf2o's median velocity over the vote's window with the vote.

    The median, so one wild reading cannot carry the window away from the vote
    and drop the good readings that follow it; a steady drag still moves it.

    readings holds (t, vx, vy) of rf2o's recent readings.
    """
    inside = [(vx, vy) for t, vx, vy in readings if vote.start <= t <= vote.end]
    if not inside:
        return None
    vx = median(v[0] for v in inside)
    vy = median(v[1] for v in inside)
    return Referee(vote, math.hypot(vx - vote.vx, vy - vote.vy))


@dataclass
class TrustLimits:
    min_beams: int
    full_trust_beams: int
    max_scan_gap: float
    max_clump_share: float
    agree_speed: float
    max_reading_gap: float


def verdict(
    dt,
    velocity,
    beams,
    shares,
    referee: Referee | None,
    limits: TrustLimits,
) -> str:
    """Why a reading is dropped, or which trust tier it gets.

    velocity is the reading's (vx, vy). beams and shares hold the beam count and
    largest-clump share of the two scans rf2o matched; None if one was not seen.
    referee is None when too few objects voted.
    """
    # When a scan match fails, rf2o still publishes: the previous estimate, with its stamp.
    if dt <= 0.0:
        return "dropped: repeated stamp"
    if dt > limits.max_scan_gap:
        return f"dropped: {dt:.1f} s since the last reading"
    if None in beams:
        return "dropped: no matching scan"
    # rf2o's estimate comes from matching two scans, so it is only as good as the sparser one.
    if min(beams) < limits.min_beams:
        return "dropped: too few beams"
    if referee is not None:
        vote = referee.vote
        if referee.gap > limits.agree_speed:
            return f"dropped: rf2o's last second is {referee.gap:.2f} m/s off what {vote.agreeing} objects say"
        # The window can agree on average around one wild reading.
        gap = math.hypot(velocity[0] - vote.vx, velocity[1] - vote.vy)
        if gap > limits.max_reading_gap:
            return f"dropped: this reading is {gap:.2f} m/s off what {vote.agreeing} objects say"
    # Agreeing objects outrank the clump check: a dock fills the scan but does not move.
    if referee is None and max(shares) > limits.max_clump_share:
        return f"low trust: one object is {max(shares):.0%} of the scan"
    return "full trust" if min(beams) >= limits.full_trust_beams else "low trust"


class Rf2oCovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__("rf2o_covariance_relay")

        self.declare_parameter("input_topic", "/odom_rf2o")
        self.declare_parameter("output_topic", "/lidar/odom")
        self.declare_parameter("scan_topic", "/lidar/above_water_scan")
        self.declare_parameter("trust_topic", "/lidar/odom_trust")
        self.declare_parameter("cluster_topic", "/lidar/vote_clusters")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("base_frame", "base_link")

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
        # rf2o goes with the majority of beams, so past half one object can steer it.
        self.declare_parameter("max_clump_share", 0.5)
        self.declare_parameter("clump_range_jump", 0.5)

        # RoboBuoys are 0.43 m across; anything much bigger may be a boat or the dock.
        self.declare_parameter("max_object_size", 1.0)
        # Centres wobble a few cm per scan; over 1 s that is a few cm/s of speed.
        self.declare_parameter("vote_window", 1.0)
        # Wide enough for 1.5 m/s over a 0.5 s gap in sightings; buoy gates are ~2 m apart.
        self.declare_parameter("match_gate", 1.0)
        self.declare_parameter("min_agreeing_objects", 3)
        self.declare_parameter("agree_speed", 0.3)
        self.declare_parameter("max_vote_age", 0.5)
        # Twice agree_speed: the vote averages the last second, so it lags under acceleration.
        self.declare_parameter("max_reading_gap", 0.6)

        # Optional: rewrite frame ids so rf2o's own "odom" frame does not
        # collide with the frame the EKF owns. Leave empty to pass through.
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "")

        def param(name):
            return self.get_parameter(name).value

        self._pose_var = list(param("pose_variances"))
        self._twist_var = list(param("twist_variances"))
        self._sparse_twist_var = list(param("sparse_twist_variances"))
        self._limits = TrustLimits(
            min_beams=param("min_beams"),
            full_trust_beams=param("full_trust_beams"),
            max_scan_gap=param("max_scan_gap"),
            max_clump_share=param("max_clump_share"),
            agree_speed=param("agree_speed"),
            max_reading_gap=param("max_reading_gap"),
        )
        self._clump_range_jump = param("clump_range_jump")
        self._base_frame = param("base_frame")
        self._max_object_size = param("max_object_size")
        self._max_vote_age = param("max_vote_age")
        self._frame_id = param("frame_id")
        self._child_frame_id = param("child_frame_id")

        window = param("vote_window")
        self._heading = GyroHeading(kept_s=3 * window)
        self._vote = ObjectVote(
            heading=self._heading,
            window=window,
            match_gate=param("match_gate"),
            min_agreeing=param("min_agreeing_objects"),
            agree_speed=self._limits.agree_speed,
        )
        self._readings: deque[tuple[float, float, float]] = deque()
        self._readings_kept_s = 2 * window
        self._referee: Referee | None = None

        self._scans_by_stamp: dict[int, tuple[int, float]] = {}
        self._previous: Odometry | None = None

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        out_topic = param("output_topic")
        in_topic = param("input_topic")
        scan_topic = param("scan_topic")

        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self._trust_pub = self.create_publisher(
            DiagnosticStatus,
            param("trust_topic"),
            10,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MarkerArray,
            param("cluster_topic"),
            self._on_clusters,
            10,
        )
        self.create_subscription(
            Imu,
            param("imu_topic"),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, in_topic, self._on_odom, 10)

        self.get_logger().info(
            f"relaying {in_topic} -> {out_topic}, trust from beams on {scan_topic}",
        )

    def _on_scan(self, scan: LaserScan) -> None:
        self._scans_by_stamp[stamp_ns(scan)] = (
            beam_count(scan),
            largest_clump_share(scan, self._clump_range_jump),
        )
        while len(self._scans_by_stamp) > SCANS_KEPT:
            del self._scans_by_stamp[next(iter(self._scans_by_stamp))]

    def _on_imu(self, msg: Imu) -> None:
        # The IMU is mounted turned about z, which leaves its yaw rate unchanged.
        self._heading.add(stamp_ns(msg) * 1e-9, msg.angular_velocity.z)

    def _on_clusters(self, msg: MarkerArray) -> None:
        boxes = [m for m in msg.markers if m.action == Marker.ADD]
        if not boxes:
            return
        try:
            mount = self._tf.lookup_transform(
                self._base_frame,
                boxes[0].header.frame_id,
                Time(),
            )
        except TransformException as e:
            self.get_logger().warning(
                f"no lidar mount transform yet: {e}",
                throttle_duration_sec=5.0,
            )
            return
        points = []
        for box in boxes:
            if max(box.scale.x, box.scale.y) > self._max_object_size:
                continue
            centre = do_transform_pose(box.pose, mount).position
            points.append((centre.x, centre.y))

        vote = self._vote.update(stamp_ns(boxes[0]) * 1e-9, points)
        if vote is not None:
            self._referee = referee_from(vote, self._readings)

    def _on_odom(self, msg: Odometry) -> None:
        previous, self._previous = self._previous, msg
        if previous is None:
            self._report(
                msg,
                [None, None],
                [None, None],
                None,
                "dropped: first reading",
                sd=None,
            )
            return
        t = stamp_ns(msg) * 1e-9
        dt = (stamp_ns(msg) - stamp_ns(previous)) * 1e-9
        scans = [self._scans_by_stamp.get(stamp_ns(m)) for m in (previous, msg)]
        beams = [None if s is None else s[0] for s in scans]
        shares = [None if s is None else s[1] for s in scans]

        if 0.0 < dt <= self._limits.max_scan_gap:
            msg.twist.twist.linear.y = sideways_speed(previous, msg, dt)
            self._readings.append(
                (t, msg.twist.twist.linear.x, msg.twist.twist.linear.y),
            )
            while t - self._readings[0][0] > self._readings_kept_s:
                self._readings.popleft()

        referee = self._referee
        if referee is not None and abs(t - referee.vote.end) > self._max_vote_age:
            referee = None
        velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        result = verdict(dt, velocity, beams, shares, referee, self._limits)
        if result.startswith("dropped"):
            if result == "dropped: no matching scan":
                self.get_logger().warning(
                    "no scan matches this odometry stamp, dropping it",
                    throttle_duration_sec=5.0,
                )
            self._report(msg, beams, shares, referee, result, sd=None)
            return

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
        self._report(msg, beams, shares, referee, result, sd=math.sqrt(twist_var[0]))

    def _report(
        self,
        msg: Odometry,
        beams,
        shares,
        referee,
        result: str,
        sd: float | None,
    ) -> None:
        values = {
            "stamp": f"{stamp_ns(msg) * 1e-9:.3f}",
            "beams_previous": str(beams[0]),
            "beams_current": str(beams[1]),
            "largest_clump": "none" if None in shares else f"{max(shares):.2f}",
            "objects_agreeing": (
                "none" if referee is None else str(referee.vote.agreeing)
            ),
            "gap_to_objects": "none" if referee is None else f"{referee.gap:.3f}",
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
