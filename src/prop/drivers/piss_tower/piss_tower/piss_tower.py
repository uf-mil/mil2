#!/usr/bin/env python3

"""ROS2 node to set/unset manual mode on the RF Kill Controller via USB."""

import struct

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

# ---------------------------------------------------------------------------- #
#                                   FUNCTIONS                                  #
# ---------------------------------------------------------------------------- #


def fletcher16(data: bytes) -> int:
    """Compute the Fletcher-16 checksum of a byte sequence."""
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


def make_msg(class_id: int, subclass_id: int, payload: bytes) -> bytes:
    """Create a message packet to send via USB."""
    sync = b"\x37\x01"
    cid_byte = struct.pack("<B", class_id)
    scid_byte = struct.pack("<B", subclass_id)
    length = struct.pack("<H", len(payload))

    data1 = cid_byte + scid_byte + length + payload
    checksum = fletcher16(data1)
    checksum_bytes = struct.pack("<H", checksum)

    return sync + data1 + checksum_bytes


def msg_to_string(msg: bytes) -> str:
    """Convert a message to a hex string for logging."""
    return " ".join(f"{byte:02x}" for byte in msg)


# ---------------------------------------------------------------------------- #
#                                    NODE                                      #
# ---------------------------------------------------------------------------- #


class ManualModeNode(Node):

    CLASS_ID = 0x10
    SUBCLASS_ID = 0x02  # Set move / manual mode subclass

    def __init__(self):
        super().__init__("rf_kill_manual_mode")

        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6635C08CB725C2C-if00",
        )
        self.declare_parameter("baud_rate", 115200)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to serial port: {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            raise

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.subscription = self.create_subscription(
            Bool,
            "/manual_control",
            self.manual_control_callback,
            qos,
        )

        self.get_logger().info("ManualModeNode started, listening on /manual_control")

    def manual_control_callback(self, msg: Bool):
        """Send manual/autonomous mode command over serial when topic updates."""
        is_manual = 1 if msg.data else 0
        payload = struct.pack("<B", is_manual)
        packet = make_msg(self.CLASS_ID, self.SUBCLASS_ID, payload)

        mode_str = "MANUAL" if is_manual else "AUTONOMOUS"
        self.get_logger().info(
            f"Setting mode: {mode_str} | Packet: {msg_to_string(packet)}",
        )

        try:
            self.ser.write(packet)
            response = self.ser.read(1000)
            if response:
                self.get_logger().info(f"Response: {msg_to_string(response)}")
            else:
                self.get_logger().warn("No response received from device")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        """Clean up serial connection on shutdown."""
        if self.ser.is_open:
            self.ser.read(1000)  # drain any remaining messages
            self.ser.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #


def main(args=None):
    rclpy.init(args=args)
    node = ManualModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
