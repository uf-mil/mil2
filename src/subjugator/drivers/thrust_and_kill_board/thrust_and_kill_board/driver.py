#! /usr/bin/env python3

from __future__ import annotations

import rclpy
from electrical_protocol import AckPacket, NackPacket, Packet, SerialDeviceNode
from std_msgs.msg import String
from subjugator_msgs.msg import ThrusterEfforts

from thrust_and_kill_board.packets import (
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    ThrusterId,
    ThrustSetPacket,
)

PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6614C311B66C338-if00"
BAUDRATE = 115200


def msg_to_string(msg):
    """Convert a message to a string to print."""
    return " ".join(f"{byte:02x}" for byte in msg)


class ThrustAndKillNode(
    SerialDeviceNode[
        ThrustSetPacket | HeartbeatSetPacket,
        HeartbeatReceivePacket | AckPacket | NackPacket,
    ],
):

    killed: bool = True

    def __init__(self):
        super().__init__("thrust_and_kill_node", PORT, BAUDRATE)
        # TODO (kill, cbrxyz): uf-mil/mil2#85
        self.thruster_sub = self.create_subscription(
            ThrusterEfforts,
            "thruster_efforts",
            self._thruster_efforts_cb,
            10,
        )
        self.test_pub = self.create_publisher(String, "thrust_set", 10)

    def _thruster_efforts_cb(self, msg: ThrusterEfforts):
        self.send_packet(ThrustSetPacket(ThrusterId.FLH, min(msg.thrust_flh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.FRH, min(msg.thrust_frh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.FLV, min(msg.thrust_flv, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.FRV, min(msg.thrust_frv, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BLH, min(msg.thrust_blh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BRH, min(msg.thrust_brh, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BLV, min(msg.thrust_blv, 1)))
        self.send_packet(ThrustSetPacket(ThrusterId.BRV, min(msg.thrust_brv, 1)))

    def on_packet_received(self, packet: Packet):
        self.get_logger().error(f"Received packet: {packet}")


def main(args=None):
    rclpy.init(args=args)
    thrust_kill_node = ThrustAndKillNode()
    rclpy.spin(thrust_kill_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
