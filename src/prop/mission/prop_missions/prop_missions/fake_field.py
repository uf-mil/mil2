#!/usr/bin/env python3
"""
Safe Passage without the boat or Gazebo: a stand-in for the boat, the planner
and beacon_node, and a judge.

    ros2 run prop_missions fake_field --ros-args -p layout:=zigzag
    ros2 launch prop_missions safe_passage.launch.py

It also stands in for the OCS, sending RunStart until the mission starts. The boat drives straight at each /goal_pose at 1 m/s, so obstacles are not
avoided. A buoy's light becomes known once the boat has been within 10 m of it
for 2 s since /beacons/reset. GPS and IMU follow the boat.

It also stands in for localization, for boat_timeout and survey: the odometry
goes out on /odometry/filtered/global too, map -> odom is the identity, and
/fromLL turns a latitude and longitude into map.

When the mission says the task is over, it prints how the run would score: how
far round each circle went, which side each red and green buoy passed on, and
whether the last report had every buoy right.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from mil_msgs.msg import PerceptionObject, PerceptionObjectArray
from nav_msgs.msg import Odometry
from prop_ocs_interfaces.msg import SafePassageReport
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String, UInt32
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster

from prop_missions.geo import EARTH_RADIUS, offset_latlon

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
DATUM = (1.2891, 103.8612)
SPEED = 1.0  # m/s
SEEN_WITHIN, SEEN_FOR = 10.0, 2.0  # m, s
NEAR = 4.0  # m: goals this close to the entry or exit are its circle

LAYOUTS = {
    "straight": [
        (10, 0, "FLASHING_BLUE"),
        (16, -3, "FLASHING_RED"),
        (16, 3, "FLASHING_GREEN"),
        (23, -2, "FLASHING_RED"),
        (24, 4, "FLASHING_GREEN"),
        (30, -1, "FLASHING_RED"),
        (30.5, 5, "FLASHING_GREEN"),
        (20, 9, "OFF"),
        (27, -7, "OFF"),
        (38, 2, "STEADY_BLUE"),
    ],
    "zigzag": [
        (12, 6, "FLASHING_BLUE"),
        (18, 3, "FLASHING_GREEN"),
        (22, -2, "FLASHING_RED"),
        (26, 5, "FLASHING_GREEN"),
        (30, 0, "FLASHING_RED"),
        (34, -6, "OFF"),
        (24, 11, "OFF"),
        (17, -6, "OFF"),
        (8, -8, "OFF"),
        (40, 3, "STEADY_BLUE"),
    ],
}


def winding(points, center):
    """Degrees turned about center along points; positive is counter-clockwise."""
    angles = [math.atan2(y - center[1], x - center[0]) for x, y in points]
    return math.degrees(
        sum(math.remainder(b - a, 2 * math.pi) for a, b in zip(angles, angles[1:])),
    )


def side_passed(points, buoy):
    """'port' or 'starboard' where the path comes closest to buoy, or None."""
    best = None
    for p, q in zip(points, points[1:]):
        d = q - p
        if d @ d < 1e-9:
            continue
        near = p + np.clip((buoy - p) @ d / (d @ d), 0.0, 1.0) * d
        cross = d[0] * (buoy - near)[1] - d[1] * (buoy - near)[0]
        if best is None or np.linalg.norm(buoy - near) < best[0]:
            best = (np.linalg.norm(buoy - near), "port" if cross > 0 else "starboard")
    return best[1] if best and best[0] < 8.0 else None


class FakeField(Node):
    def __init__(self):
        super().__init__("fake_field")
        self.buoys = LAYOUTS[self.declare_parameter("layout", "straight").value]
        self.frame = self.declare_parameter("frame", "map").value
        self.seen = [0.0] * len(self.buoys)
        self.reset = False
        self.x = self.y = self.yaw = 0.0
        self.goal = None
        self.t = 0.0
        self.track, self.goals = [], []
        self.task = None
        self.last_report = None

        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered/local", 10)
        self.global_pub = self.create_publisher(
            Odometry,
            "/odometry/filtered/global",
            10,
        )
        self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.beacons_pub = self.create_publisher(PerceptionObjectArray, "/beacons", 10)
        self.run_start_pub = self.create_publisher(UInt32, "/prop/run_start", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(String, "/mission/current_task", self.on_task, LATCHED)
        self.create_subscription(
            SafePassageReport,
            "/prop/safe_passage_report",
            lambda msg: setattr(self, "last_report", msg),
            10,
        )
        self.create_service(SetBool, "/planner/enable", self.on_enable)
        self.create_service(Trigger, "/beacons/reset", self.on_reset)
        self.create_service(FromLL, "/fromLL", self.on_from_ll)
        identity = TransformStamped()
        identity.header.frame_id, identity.child_frame_id = "map", "odom"
        identity.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(identity)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.send_run_start)

    def send_run_start(self):
        if self.task is None:
            self.run_start_pub.publish(UInt32(data=1))

    def on_enable(self, _, response):
        response.success = True
        return response

    def on_reset(self, _, response):
        self.reset = True
        self.seen = [0.0] * len(self.buoys)
        response.success = True
        return response

    def on_from_ll(self, request, response):
        lat, lon = request.ll_point.latitude, request.ll_point.longitude
        response.map_point.y = math.radians(lat - DATUM[0]) * EARTH_RADIUS
        response.map_point.x = (
            math.radians(lon - DATUM[1])
            * EARTH_RADIUS
            * math.cos(math.radians(DATUM[0]))
        )
        return response

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goals.append((self.t, *self.goal))

    def on_task(self, msg):
        if self.task == "SAFE_PASSAGE" and msg.data == "NONE":
            self.score()
        self.task = msg.data

    def tick(self):
        dt = 0.1
        self.t += dt
        if self.goal is not None:
            dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
            distance = math.hypot(dx, dy)
            if distance > 0.05:
                step = min(distance, SPEED * dt)
                self.x += step * dx / distance
                self.y += step * dy / distance
                self.yaw = math.atan2(dy, dx)
        self.track.append((self.t, self.x, self.y))
        stamp = self.get_clock().now().to_msg()
        q = (math.sin(self.yaw / 2), math.cos(self.yaw / 2))

        odom = Odometry()
        odom.header.frame_id = self.frame
        odom.header.stamp = stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        self.odom_pub.publish(odom)
        self.global_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.orientation.z, imu.orientation.w = q
        self.imu_pub.publish(imu)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude = offset_latlon(*DATUM, self.x, self.y)
        self.fix_pub.publish(fix)

        beacons = PerceptionObjectArray()
        for i, (bx, by, state) in enumerate(self.buoys):
            if self.reset and math.hypot(bx - self.x, by - self.y) < SEEN_WITHIN:
                self.seen[i] += dt
            known = self.seen[i] >= SEEN_FOR
            o = PerceptionObject()
            o.header.frame_id = self.frame
            o.header.stamp = stamp
            o.id = i + 1
            o.pose.position.x, o.pose.position.y = float(bx), float(by)
            o.scale.x = o.scale.y = 0.5
            o.labeled_classification = state if known else "UNKNOWN"
            o.classification_confidence = 1.0 if known else 0.0
            beacons.objects.append(o)
        self.beacons_pub.publish(beacons)

    def score(self):
        log = self.get_logger().info
        track = np.array([(x, y) for _, x, y in self.track])
        times = np.array([t for t, _, _ in self.track])
        entry = next(b for b in self.buoys if b[2] == "FLASHING_BLUE")
        exit_ = next(b for b in self.buoys if b[2] == "STEADY_BLUE")

        def circle_times(center):
            """First and last time of the longest run of goals near center: its circle."""
            best, run = [], []
            for t, x, y in self.goals:
                if math.hypot(x - center[0], y - center[1]) < NEAR:
                    run.append(t)
                    best = max(best, run, key=len)
                else:
                    run = []
            return (best[0], best[-1]) if len(best) > 2 else None

        circles = {}
        for name, buoy in (("entry", entry), ("exit", exit_)):
            span = circle_times(buoy)
            turned = 0.0
            if span:
                turned = winding(
                    track[(times >= span[0]) & (times <= span[1] + 1.0)],
                    buoy,
                )
            circles[name] = span
            want = "clockwise" if name == "entry" else "counter-clockwise"
            ok = turned <= -360 if name == "entry" else turned >= 360
            log(
                f"{name} circled {abs(turned):.0f} deg {'clockwise' if turned < 0 else 'counter-clockwise'}: {'OK' if ok else 'WRONG, wanted ' + want}",
            )

        if circles["entry"] and circles["exit"]:
            between = track[
                (times >= circles["entry"][1]) & (times <= circles["exit"][0])
            ]
            for x, y, state in self.buoys:
                if state in ("FLASHING_RED", "FLASHING_GREEN"):
                    side = side_passed(between, np.array([x, y], float))
                    want = "starboard" if state == "FLASHING_RED" else "port"
                    log(
                        f"{state} at ({x}, {y}) passed to {side}: {'OK' if side == want else 'WRONG'}",
                    )

        report = self.last_report
        if report is None:
            log("no report")
            return
        right = 0
        for b in report.buoys:
            north = math.radians(b.position.latitude - DATUM[0]) * EARTH_RADIUS
            east = (
                math.radians(b.position.longitude - DATUM[1])
                * EARTH_RADIUS
                * math.cos(math.radians(DATUM[0]))
            )
            truth = min(self.buoys, key=lambda t: math.hypot(t[0] - east, t[1] - north))
            right += (
                truth[2] == b.state
                and math.hypot(truth[0] - east, truth[1] - north) < 1.0
            )
        log(
            f"last report: {right} of {len(report.buoys)} buoys right, of {len(self.buoys)} in the field",
        )
        log(f"run took {self.t:.0f} s")


def main():
    rclpy.init()
    node = FakeField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
