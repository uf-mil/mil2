#!/usr/bin/env python3
"""Perception-driven docking. Inputs are RGB, CameraInfo, LiDAR and estimated odom.

No simulator ground truth, world files, dock pose parameters, or bay-order tables
are available to this node. The separate evaluator owns those.
"""

import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from prop_vision.dock_perception import color_centers, detect_bay, ray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import String

from prop_missions.ocs_protocol import Exchange, decode


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return np.array([p.x, p.y]), yaw


def settled(msg, point, yaw):
    position, heading = pose(msg)
    v = msg.twist.twist
    return (
        np.linalg.norm(position - point) < 0.2
        and abs(math.remainder(heading - yaw, 2 * math.pi)) < 0.10
        and math.hypot(v.linear.x, v.linear.y) < 0.08
        and abs(v.angular.z) < 0.05
    )


class DockingMission(Node):
    def __init__(self):
        super().__init__("docking_mission")
        self.target = "green"
        self.exchange = Exchange(
            self.declare_parameter("vehicle_id", "prop").value,
            self.declare_parameter("ack_timeout", 15.0).value,
        )
        self.ocs_status = self.create_publisher(String, "/dock/ocs/status", 10)
        self.ocs_result = self.create_publisher(String, "/dock/ocs/result", 10)
        self.create_subscription(
            String, "/dock/ocs/command", self.command,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(String, "/dock/ocs/receipt", self.receipt, 10)
        self.timeout = self.declare_parameter("timeout", 240.0).value
        self.sensor_timeout = self.declare_parameter("sensor_timeout", 1.0).value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Path, "plan", qos)
        self.status = self.create_publisher(String, "/dock/status", qos)
        self.odom = deque(maxlen=150)
        self.cloud = self.image = self.info = None
        self.cloud_origin = None
        self.bay = None
        self.candidate = None
        self.confirmations = 0
        self.since = None
        self.paused = False
        self.fresh_since = None
        self.last_detection = -100.0
        self.last_log = -100.0
        self.start = None
        self.state = "WAIT_START"
        self.create_subscription(
            Odometry,
            "odometry/filtered/global",
            lambda msg: self.odom.append(msg),
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/lidar/points",
            lambda msg: setattr(self, "cloud", msg),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            "/camera/image",
            lambda msg: setattr(self, "image", msg),
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            lambda msg: setattr(self, "info", msg),
            sensor_qos,
        )
        self.create_timer(0.2, self.step)
        self.pub.publish(Path())
        self.create_timer(0.5, self.exchange_tick)
        self.report("WAIT_START: waiting for OCS command; no docking plan")

    def command(self, message):
        action = self.exchange.command(decode(message.data))
        if action == "STARTED":
            self.start = self.seconds()
            self.state = "SEARCH"
            self.report("STARTED: accepted OCS command")
        elif action == "CANCELLED":
            self.fail("OCS cancelled task")

    def receipt(self, message):
        if self.exchange.acknowledge(decode(message.data), self.seconds()):
            self.report("RECEIVED: OCS acknowledged docking event (not official scoring)")

    def exchange_tick(self):
        outgoing = self.exchange.outgoing(self.seconds())
        if outgoing is not None:
            self.ocs_result.publish(String(data=json.dumps(outgoing)))
        msg = dict(self.exchange.envelope("status"), state=self.state,
                   receipt=self.exchange.receipt, paused=self.paused,
                   sent_at=self.seconds(), clock="ros",
                   event_id=self.exchange.result["event_id"] if self.exchange.result else None)
        self.ocs_status.publish(String(data=json.dumps(msg)))

    def seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def report(self, description):
        self.get_logger().info(description)
        self.status.publish(
            String(
                data=json.dumps(
                    {"state": self.state, "target": self.target, "detail": description},
                ),
            ),
        )

    def fail(self, reason):
        self.exchange.close()
        self.state = "FAIL"
        self.pub.publish(Path())
        self.report("FAIL: " + reason + "; cleared plan")

    def plan(self, point):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        waypoint = PoseStamped()
        waypoint.header = msg.header
        waypoint.pose.position.x, waypoint.pose.position.y = map(float, point)
        waypoint.pose.orientation.z = math.sin(self.bay.yaw / 2)
        waypoint.pose.orientation.w = math.cos(self.bay.yaw / 2)
        msg.poses = [waypoint]
        self.pub.publish(msg)

    def sample(self):
        # Sensor stamps are matched against odometry, not callback arrival time.
        cloud_odom = min(self.odom, key=lambda msg: abs(stamp(msg) - stamp(self.cloud)))
        image_odom = min(self.odom, key=lambda msg: abs(stamp(msg) - stamp(self.image)))
        if (
            max(
                abs(stamp(cloud_odom) - stamp(self.cloud)),
                abs(stamp(image_odom) - stamp(self.image)),
            )
            > 0.2
        ):
            return None
        xyz = read_points_numpy(
            self.cloud,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[
            (xyz[:, 2] > -0.38)
            & (xyz[:, 2] < 0.18)
            & (np.linalg.norm(xyz[:, :2], axis=1) > 0.8)
            & (np.linalg.norm(xyz[:, :2], axis=1) < 40)
        ]
        origin, yaw = pose(cloud_odom)
        # Keep the origin paired with this cloud. Comparing these transformed
        # points against newer odometry creates false proximity alarms while
        # the boat moves and rendering delivers an older cloud.
        self.cloud_origin = origin
        c, s = math.cos(yaw), math.sin(yaw)
        points = xyz[:, :2] @ np.array([[c, s], [-s, c]]) + origin
        if self.image.encoding not in ("rgb8", "bgr8"):
            self.fail("unsupported camera encoding " + self.image.encoding)
            return None
        rgb = np.frombuffer(self.image.data, dtype=np.uint8).reshape(
            self.image.height,
            self.image.step,
        )
        rgb = rgb[:, : self.image.width * 3].reshape(
            self.image.height,
            self.image.width,
            3,
        )
        if self.image.encoding == "bgr8":
            rgb = rgb[:, :, ::-1].copy()
        pixels = color_centers(rgb)
        origin, yaw = pose(image_odom)
        camera = origin + 0.1 * np.array([math.cos(yaw), math.sin(yaw)])
        return points, pixels, float(self.info.k[0]), float(self.info.k[2]), camera, yaw

    def visible_target(self, sample):
        points, pixels, fx, cx, camera, yaw = sample
        if len(pixels.get(self.target, [])) != 1:
            return False
        direction = ray(pixels[self.target][0], fx, cx, yaw)
        denom = direction @ self.bay.inward
        if denom < 0.3:
            return False
        hit = camera + direction * (
            ((self.bay.beacon - camera) @ self.bay.inward) / denom
        )
        return (
            np.linalg.norm(hit - self.bay.beacon) < 0.45
            and len(points) > 0
            and np.min(np.linalg.norm(points - hit, axis=1)) < 0.4
        )

    def step(self):
        now = self.seconds()
        if self.state in ("FAIL", "WAIT_START"):
            return
        if self.start is None:
            if now <= 0:
                return
            self.start = now
        if now - self.start > self.timeout and self.state != "DOCKED":
            self.fail("mission timeout")
            return
        available = bool(self.odom) and all(
            msg is not None for msg in (self.cloud, self.image, self.info)
        )
        if not available:
            if now - self.last_log > 5:
                self.get_logger().info(
                    f"Waiting: odom={bool(self.odom)} lidar={self.cloud is not None} image={self.image is not None} info={self.info is not None}",
                )
                self.last_log = now
            return
        ages = [now - stamp(msg) for msg in (self.odom[-1], self.cloud, self.image)]
        if any(not -0.3 <= age < self.sensor_timeout for age in ages):
            if self.state != "SEARCH":
                if not self.paused:
                    self.pub.publish(Path())
                    self.paused = True
                    self.since = None
                    self.report(
                        f"PAUSED: stale odom/lidar/image ages={ages}; plan cleared",
                    )
                self.fresh_since = None
            return
        if self.paused:
            self.fresh_since = now if self.fresh_since is None else self.fresh_since
            if now - self.fresh_since < 1.0:
                return
            self.paused = False
            self.plan(self.bay.stage if self.state == "ALIGN" else self.bay.goal)
            self.report("RESUMED: sensors fresh for one second")
        sample = self.sample()
        if sample is None:
            if self.state != "SEARCH" and self.state != "FAIL":
                self.fail("sensor/odometry timestamps do not align")
            return
        points = sample[0]
        if (
            len(points)
            and self.cloud_origin is not None
            and np.min(np.linalg.norm(points - self.cloud_origin, axis=1)) < 0.72
        ):
            self.fail("LiDAR obstacle inside hull clearance margin")
            return
        if self.state == "SEARCH":
            if now - self.last_detection < 1.0:
                return
            self.last_detection = now
            found = detect_bay(*sample)
            if found is None:
                self.confirmations = 0
                if now - self.last_log > 5:
                    self.get_logger().info(
                        f"SEARCH: {len(points)} LiDAR points; visible colors {list(sample[1])}; waiting for supported bay geometry",
                    )
                    self.last_log = now
                return
            consistent = (
                self.candidate is not None
                and np.linalg.norm(found.goal - self.candidate.goal) < 0.25
                and found.inward @ self.candidate.inward > 0.995
                and found.number == self.candidate.number
            )
            self.confirmations = self.confirmations + 1 if consistent else 1
            self.candidate = found
            if self.confirmations < 3:
                return
            self.bay = found
            self.state = "ALIGN"
            self.plan(found.stage)
            self.report(
                f"ALIGN: detected {self.target} bay; stage={found.stage.round(2).tolist()}, goal={found.goal.round(2).tolist()}, heading={math.degrees(found.yaw):.1f}, width={found.width:.2f}",
            )
            return
        target = self.bay.stage if self.state == "ALIGN" else self.bay.goal
        good = settled(self.odom[-1], target, self.bay.yaw)
        if self.state == "ALIGN":
            good = good and self.visible_target(sample)
        self.since = (now if self.since is None else self.since) if good else None
        if self.since is not None and now - self.since >= 3 and self.state != "DOCKED":
            self.since = None
            if self.state == "ALIGN":
                self.state = "ENTER"
                self.plan(self.bay.goal)
                self.report("ENTER: aligned and camera/LiDAR target revalidated")
            else:
                self.state = "DOCKED"
                self.exchange.complete(self.bay.number, now)
                self.report(
                    "DOCKED: estimated pose and motion stable; independent scorer determines PASS",
                )


def main():
    rclpy.init()
    node = DockingMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # rclpy can race subscription teardown when launch shuts down the graph.
        # Preserve real callback errors while the context is still running.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
