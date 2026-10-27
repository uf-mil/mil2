"""Simulation-only OCS peer. A receipt means delivery, never official scoring."""

import json
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from prop_missions.ocs_protocol import decode, identifier


class MockOCS(Node):
    def __init__(self):
        super().__init__("mock_ocs")
        self.vehicle_id = self.declare_parameter("vehicle_id", "prop").value
        self.start_delay = self.declare_parameter("start_delay", 3.0).value
        self.drop_receipts = self.declare_parameter("drop_receipts", 0).value
        self.auto_start = self.declare_parameter("auto_start", True).value
        self.command_pub = self.create_publisher(
            String, "/dock/ocs/command",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.receipt_pub = self.create_publisher(String, "/dock/ocs/receipt", 10)
        self.create_subscription(String, "/dock/ocs/status", self.status, 10)
        self.create_subscription(String, "/dock/ocs/result", self.result, 10)
        self.command = None
        self.first_seen = None
        self.last_status = None
        self.accepted = None
        self.create_timer(1.0, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def status(self, message):
        msg = decode(message.data)
        if msg is None or msg["kind"] != "status" or msg["vehicle_id"] != self.vehicle_id:
            return
        if self.command is None and msg.get("state") == "WAIT_START" and self.now() > 0:
            self.first_seen = self.now()
            self.command = dict(version=1, kind="command", vehicle_id=self.vehicle_id,
                                session_id=msg["session_id"], run_id=uuid.uuid4().hex,
                                command_id=uuid.uuid4().hex, action="START")
        if self.command and msg["session_id"] == self.command["session_id"]:
            self.last_status = msg

    def tick(self):
        if (self.auto_start and self.command and self.last_status
                and self.last_status.get("state") == "WAIT_START"
                and self.now() - self.first_seen >= self.start_delay):
            self.command_pub.publish(String(data=json.dumps(self.command)))
            self.get_logger().info("MOCK_START: sent start command")

    def result(self, message):
        msg = decode(message.data)
        if msg is None or self.command is None or msg["kind"] != "docking_result":
            return
        if any(msg.get(k) != self.command[k] for k in ("vehicle_id", "session_id", "run_id")):
            return
        if not identifier(msg.get("event_id")) or type(msg.get("bay_number")) is not int or msg["bay_number"] not in (1, 2, 3):
            return
        # Same event must always carry identical content. A second event in this
        # single-run peer is rejected rather than counted as another docking.
        if self.accepted is not None and msg != self.accepted:
            return
        if self.accepted is None:
            self.accepted = msg
            self.get_logger().info(f"MOCK_RECEIVED: bay={msg['bay_number']} event={msg['event_id']}")
        if self.drop_receipts > 0:
            self.drop_receipts -= 1
            self.get_logger().info("MOCK_DROP: intentionally suppressing receipt")
            return
        receipt = {k: msg[k] for k in ("version", "vehicle_id", "session_id", "run_id", "event_id")}
        receipt.update(kind="receipt", status="RECEIVED")
        self.receipt_pub.publish(String(data=json.dumps(receipt)))


def main():
    rclpy.init()
    node = MockOCS()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
