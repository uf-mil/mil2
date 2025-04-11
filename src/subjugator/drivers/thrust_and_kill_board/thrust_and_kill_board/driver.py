#! /usr/bin/env python3

from __future__ import annotations

import contextlib

import rclpy
from electrical_protocol import AckPacket, NackPacket, SerialDeviceNode
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from subjugator_msgs.msg import ThrusterEfforts

from thrust_and_kill_board.packets import (
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    ThrusterId,
    ThrustSetPacket,
)

DEFAULT_PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6614C311B66C338-if00"
DEFAULT_BAUDRATE = 115200


def msg_to_string(msg):
    """Convert a message to a string to print."""
    return " ".join(f"{byte:02x}" for byte in msg)


SendPackets = ThrustSetPacket | HeartbeatSetPacket
ReceivePackets = HeartbeatReceivePacket | AckPacket | NackPacket


class ThrustAndKillNode(
    SerialDeviceNode[SendPackets, ReceivePackets],
):

    killed: bool = True
    heartbeat_timeout = Duration(seconds=1.25)
    last_heartbeat: Time

    def __init__(self):
        # Port parameter
        super().__init__("thrust_and_kill_board", None, None)
        port_description = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Serial port to connect to the thrust and kill board",
        )
        self.declare_parameter(
            "port",
            DEFAULT_PORT,
            descriptor=port_description,
        )
        baudurate_description = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description="Baudrate to connect to the thrust and kill board",
        )
        self.declare_parameter(
            "baudrate",
            DEFAULT_BAUDRATE,
            descriptor=baudurate_description,
        )
        port_val = self.get_parameter("port").get_parameter_value().string_value
        baudrate_val = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self.connect(port_val, baudrate_val)
        # TODO (kill, cbrxyz): uf-mil/mil2#85
        self.is_killed_srv = self.create_service(
            String,
            "is_killed",
            self._is_killed_cb,
        )
        self.thruster_sub = self.create_subscription(
            ThrusterEfforts,
            "thruster_efforts",
            self._thruster_efforts_cb,
            10,
        )
        self.test_pub = self.create_publisher(String, "thrust_set", 10)
        # Heartbeat
        self.last_heartbeat = self.get_clock().now()
        self.heartbeat_timer = self.create_timer(1.0, self._heartbeat_send_cb)

    def _heartbeat_send_cb(self):
        if (self.get_clock().now() - self.last_heartbeat) > self.heartbeat_timeout:
            self.get_logger().error(
                "Heartbeat timeout. Resetting thrust and kill board.",
            )
            self.reset()
            self.killed = True
        self.send_packet(HeartbeatSetPacket())

    def _is_killed_cb(self, request: String, response: String):
        if self.killed:
            response.data = "True"
        else:
            response.data = "False"
        return response

    def _thruster_efforts_cb(self, msg: ThrusterEfforts):
        if self.killed:
            return
        self.send_packet(ThrustSetPacket(ThrusterId.FLH, min(msg.thrust_flh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.FRH, min(msg.thrust_frh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.FLV, min(msg.thrust_flv, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.FRV, min(msg.thrust_frv, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BLH, min(msg.thrust_blh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BRH, min(msg.thrust_brh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BLV, min(msg.thrust_blv, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BRV, min(msg.thrust_brv, 1)))

    def reset(self):
        self.send_packet(ThrustSetPacket(ThrusterId.FLH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.FRH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.FLV, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.FRV, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BLH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BRH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BLV, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BRV, 0))

    def on_packet_received(self, packet: ReceivePackets):
        if isinstance(packet, HeartbeatReceivePacket):
            self.last_heartbeat = self.get_clock().now()
            self.killed = False
        self.get_logger().error(f"Received packet: {packet}")


def main(args=None):
    rclpy.init(args=args)
    thrust_kill_node = ThrustAndKillNode()
    with contextlib.suppress(KeyboardInterrupt):
        rclpy.spin(thrust_kill_node)
    thrust_kill_node.reset()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
