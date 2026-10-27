#!/usr/bin/env python3
"""Independent simulation scorer. Never publishes navigation commands."""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from prop_missions.ocs_protocol import decode


class Evaluator(Node):
    def __init__(self):
        super().__init__("docking_evaluator")
        self.dx = self.declare_parameter("dock_x", 0.0).value
        self.dy = self.declare_parameter("dock_y", 0.0).value
        self.yaw = self.declare_parameter("dock_yaw", 0.0).value
        self.colors = list(
            self.declare_parameter("colors", ["red", "red", "green"]).value,
        )
        self.target = "green"
        self.timeout = self.declare_parameter("timeout", 240.0).value
        seed = self.declare_parameter("seed", 1).value
        self.goal_y = [3.0, 0.0, -3.0][self.colors.index(self.target)]
        self.ocs_state = None
        self.docking_result = None
        self.ocs_receipt = None
        self.start_command = None
        self.wait_session = None
        self.last_diagnostic = -100.0
        self.create_subscription(String, "/dock/ocs/status", self.exchange_status, 10)
        self.create_subscription(String, "/dock/ocs/result", self.result_message, 10)
        self.create_subscription(String, "/dock/ocs/receipt", self.receipt_message, 10)
        self.create_subscription(
            String, "/dock/ocs/command", self.command_message,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.start = None
        self.truth = None
        self.state = "SEARCH"
        self.since = None
        self.result = None
        self.minimum_clearance = float("inf")
        self.create_subscription(
            Odometry, "/sim/ground_truth", lambda msg: setattr(self, "truth", msg), 10,
        )
        self.create_subscription(
            String,
            "/dock/status",
            self.status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(0.1, self.step)
        self.get_logger().info(
            f"FIXTURE seed={seed} translation=({self.dx:.2f},{self.dy:.2f}) yaw={self.yaw:.3f} colors={self.colors}",
        )

    def seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def status(self, msg):
        self.state = json.loads(msg.data)["state"]

    def exchange_status(self, message):
        msg = decode(message.data)
        if msg and msg["kind"] == "status" and msg["vehicle_id"] == "prop":
            self.ocs_state = msg
            if msg.get("state") == "WAIT_START":
                self.wait_session = msg["session_id"]

    def result_message(self, message):
        msg = decode(message.data)
        if msg and msg["kind"] == "docking_result":
            self.docking_result = msg

    def receipt_message(self, message):
        msg = decode(message.data)
        if msg and msg["kind"] == "receipt":
            self.ocs_receipt = msg

    def command_message(self, message):
        msg = decode(message.data)
        if msg and msg["kind"] == "command" and msg.get("action") == "START":
            self.start_command = msg

    def exchange_complete(self):
        messages = [self.ocs_state, self.docking_result, self.ocs_receipt, self.start_command]
        if not all(messages):
            return False
        status, result, receipt, command = messages
        return (
            self.wait_session == status["session_id"]
            and all(all(m.get(k) == status.get(k) for k in ("vehicle_id", "session_id", "run_id")) for m in messages)
            and bool(status.get("run_id"))
            and status.get("receipt") == receipt.get("status") == "RECEIVED"
            and status.get("event_id") == result.get("event_id") == receipt.get("event_id")
            and bool(result.get("event_id"))
            and result.get("bay_number") == self.colors.index("green") + 1
        )

    def finish(self, success, reason):
        self.result = success
        self.get_logger().info(
            f'{"PASS" if success else "FAIL"}: {reason}; minimum geometric hull clearance={self.minimum_clearance:.3f} m',
        )

    def step(self):
        now = self.seconds()
        if self.result is not None:
            return
        if self.start is None:
            if now <= 0:
                return
            self.start = now
        if self.state == "FAIL" or now - self.start > self.timeout + 5:
            self.finish(False, "mission failed or timed out")
            return
        if self.ocs_state and self.ocs_state.get("receipt") == "TIMED_OUT":
            self.finish(False, "OCS receipt timed out")
            return
        if self.truth is None:
            return
        msg = self.truth
        if now - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9 > 1:
            self.finish(False, "ground truth stale")
            return
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = (
            c * (p.x - self.dx) + s * (p.y - self.dy),
            -s * (p.x - self.dx) + c * (p.y - self.dy),
        )
        radius = math.hypot(0.5, 0.3)
        rectangles = [(11.75, 12.25, -5, 5)] + [
            (8, 12, yy - 0.2, yy + 0.2) for yy in (-4.5, -1.5, 1.5, 4.5)
        ]
        clearance = min(
            math.hypot(max(xmin - x, 0, x - xmax), max(ymin - y, 0, y - ymax)) - radius
            for xmin, xmax, ymin, ymax in rectangles
        )
        self.minimum_clearance = min(self.minimum_clearance, clearance)
        if clearance < 0:
            self.finish(False, "dock clearance violated")
            return
        heading = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z),
        )
        good = (
            self.state == "DOCKED"
            and math.hypot(x - 10.5, y - self.goal_y) < 0.3
            and abs(math.remainder(heading - self.yaw, 2 * math.pi)) < 0.12
            and math.hypot(v.linear.x, v.linear.y) < 0.08
            and abs(v.angular.z) < 0.05
        )
        if self.state == "DOCKED" and now - self.last_diagnostic >= 10:
            self.last_diagnostic = now
            self.get_logger().info(
                f"VERIFY: physical={good} exchange={self.exchange_complete()} "
                f"wait_seen={bool(self.wait_session)} start_seen={bool(self.start_command)} "
                f"result_seen={bool(self.docking_result)} receipt_seen={bool(self.ocs_receipt)}"
            )
        self.since = (now if self.since is None else self.since) if good else None
        if self.since is not None and now - self.since >= 3 and self.exchange_complete():
            self.finish(
                True, f"green bay {self.colors.index('green') + 1} verified; WAIT_START -> docking -> report -> receipt verified",
            )


def main():
    rclpy.init()
    node = Evaluator()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = node.result
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if result else 1)


if __name__ == "__main__":
    main()
