#! /usr/bin/env python3

from __future__ import annotations

import contextlib
import time

import rclpy
from electrical_protocol import AckPacket, NackPacket, Packet, SerialDeviceNode
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String
from std_srvs.srv import Empty
from subjugator_msgs.msg import ThrusterEfforts

from thrust_and_kill_board.packets import (
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    KillReceivePacket,
    KillSetPacket,
    KillStatus,
    ThrusterId,
    ThrustSetPacket,
)

DEFAULT_PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E4623C80DF43092D-if00"
DEFAULT_BAUDRATE = 115200


def msg_to_string(msg):
    """Convert a message to a string to print."""
    return " ".join(f"{byte:02x}" for byte in msg)


class ThrustAndKillNode(
    SerialDeviceNode[
        ThrustSetPacket | HeartbeatSetPacket | KillSetPacket,
        HeartbeatReceivePacket | AckPacket | NackPacket | KillReceivePacket,
    ],
):

    killed: bool = True

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
        self.thruster_sub = self.create_subscription(
            ThrusterEfforts,
            "thruster_efforts",
            self._thruster_efforts_cb,
            10,
        )

        self.thruster_heartbeat = self.create_publisher(
            String,
            "thruster_heartbeat",
            10,
        )

        self.thruster_messages = self.create_publisher(
            String,
            "thruster_messages",
            10,
        )

        self.create_service(Empty, "kill", self._set_kill)
        self.create_service(Empty, "unkill", self._unset_kill)

        self.send_heartbeat_timer = self.create_timer(0.9, self._send_heartbeat)
        self.check_heartbeat_timer = self.create_timer(0.5, self._check_heartbeat)
        self.last_heartbeat_time = time.monotonic()
        self.heartbeat_timedout = False

    def _thruster_efforts_cb(self, msg: ThrusterEfforts):
        if not self.heartbeat_timedout:
            self.send_packet(ThrustSetPacket(ThrusterId.FLH, min(msg.thrust_flh, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.FRH, min(msg.thrust_frh, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.FLV, min(msg.thrust_flv, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.FRV, min(msg.thrust_frv, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.BLH, min(msg.thrust_blh, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.BRH, min(msg.thrust_brh, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.BLV, min(msg.thrust_blv, 1)))
            self.send_packet(ThrustSetPacket(ThrusterId.BRV, min(msg.thrust_brv, 1)))
        else:
            self.get_logger().error(
                "Cannot set thrusters. Thrust board heartbeat timed out!",
            )

    def reset(self):
        self.send_packet(ThrustSetPacket(ThrusterId.FLH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.FRH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.FLV, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.FRV, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BLH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BRH, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BLV, 0))
        self.send_packet(ThrustSetPacket(ThrusterId.BRV, 0))

    def on_packet_received(self, packet: Packet):
        if isinstance(packet, HeartbeatReceivePacket):
            self.last_heartbeat_time = time.monotonic()
        else:
            self.get_logger().debug(f"Thrust Board received packet: {packet}")
            packet_msg = String()
            packet_msg.data = str(packet)
            self.thruster_messages.publish(packet_msg)

        if isinstance(packet, KillReceivePacket):
            self.get_logger().error(f"Received kill packet from thrusters: {packet}")

    def _set_kill(self, request, response):
        self.get_logger().info("Received kill service request.")
        self.send_packet(KillSetPacket(True, KillStatus.SOFTWARE_REQUESTED))
        return response

    def _unset_kill(self, request, response):
        self.get_logger().info("Received unkill service request.")
        self.send_packet(KillSetPacket(False, KillStatus.SOFTWARE_REQUESTED))
        return response

    def _send_heartbeat(self):
        self.send_packet(HeartbeatSetPacket())

    def _check_heartbeat(self):
        if time.monotonic() - self.last_heartbeat_time > 1:
            self.get_logger().error(
                "Thrust board heartbeat timeout! Rejecting further thruster commands.",
            )
            self.heartbeat_timedout = True
        else:
            if not self.heartbeat_timedout:
                self.get_logger().debug("Thrust board heartbeat is live.")
            else:
                self.get_logger().error("Thrust board heartbeat re-established!")
                self.heartbeat_timedout = False


def main(args=None):
    rclpy.init(args=args)
    thrust_kill_node = ThrustAndKillNode()
    with contextlib.suppress(KeyboardInterrupt):
        rclpy.spin(thrust_kill_node)
    thrust_kill_node.reset()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
