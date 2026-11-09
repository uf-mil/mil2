#!/usr/bin/env python3
"""
The RF kill board, over its USB serial port: sets the mode, and passes on the
mode the board says it is in.

  in   /manual_control  std_msgs/Bool  true manual, false autonomous; sent to the board
  out  /rf_kill/manual  std_msgs/Bool  latched, the mode the board reports

The board speaks MIL's electrical packets: sync 37 01, class, subclass,
little-endian payload length, payload, and a Fletcher-16 of everything after the
sync. Setting the mode is class 0x10, subclass 0x02, one byte, 1 for manual.

The board is assumed to report its mode the same way: a packet of mode_class and
mode_subclass whose first payload byte is 1 for manual. That has not been seen
on the boat, so every packet is logged; read the real one off the log and set
the parameters, or report_mode:=false if the board's packets are not the mode.

Uses the standard library only, so it runs where pyserial is missing, and keeps
trying to open the port, so it can start before the board is plugged in.
"""

import contextlib
import os
import struct
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

SYNC = b"\x37\x01"
SET_MODE = (0x10, 0x02)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RETRY_EVERY = 2.0  # s


def fletcher16(data):
    sum1 = sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


def make_msg(class_id, subclass_id, payload):
    body = struct.pack("<BBH", class_id, subclass_id, len(payload)) + payload
    return SYNC + body + struct.pack("<H", fletcher16(body))


def packets(buffer):
    """
    The complete packets at the front of buffer, as (class, subclass, payload,
    checksum ok), and what is left over.
    """
    found = []
    while True:
        start = buffer.find(SYNC)
        if start < 0:
            return found, buffer[-1:]
        buffer = buffer[start:]
        if len(buffer) < 6:
            return found, buffer
        class_id, subclass_id, length = struct.unpack_from("<BBH", buffer, 2)
        end = 6 + length + 2
        if len(buffer) < end:
            return found, buffer
        (checksum,) = struct.unpack_from("<H", buffer, 6 + length)
        ok = checksum == fletcher16(buffer[2 : 6 + length])
        found.append((class_id, subclass_id, buffer[6 : 6 + length], ok))
        buffer = buffer[end:]


def hex_of(data):
    return " ".join(f"{byte:02x}" for byte in data)


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


class ManualModeNode(Node):
    def __init__(self):
        super().__init__("rf_kill_manual_mode")
        p = self.declare_parameter
        self.port = p(
            "serial_port",
            "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6635C08CB725C2C-if00",
        ).value
        self.baud = p("baud_rate", 115200).value
        self.mode_packet = (p("mode_class", 0x10).value, p("mode_subclass", 0x02).value)
        self.report_mode = p("report_mode", True).value

        self.fd = None
        self.last_try = None
        self.buffer = b""
        self.wanted = None  # the mode last asked for, sent again on reconnect
        self.reported = None

        self.mode_pub = self.create_publisher(Bool, "/rf_kill/manual", LATCHED)
        self.create_subscription(
            Bool,
            "/manual_control",
            self.on_manual_control,
            LATCHED,
        )
        self.create_timer(0.05, self.poll)
        self.get_logger().info(f"RF kill board on {self.port}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def connect(self):
        if self.last_try is not None and self.now() - self.last_try < RETRY_EVERY:
            return
        self.last_try = self.now()
        try:
            self.fd = open_port(self.port, self.baud)
        except OSError as e:
            self.get_logger().warn(
                f"cannot open {self.port}: {e}; trying again",
                throttle_duration_sec=30.0,
            )
            return
        self.buffer = b""
        self.get_logger().info(f"connected to {self.port}")
        if self.wanted is not None:
            self.send_mode(self.wanted)

    def disconnect(self, why):
        self.get_logger().error(f"lost {self.port}: {why}")
        with contextlib.suppress(OSError):
            os.close(self.fd)
        self.fd = None

    def on_manual_control(self, msg):
        self.wanted = msg.data
        if self.fd is None:
            self.get_logger().warn(
                f"board not connected; will set {self.mode_name(msg.data)} once it is",
            )
            return
        self.send_mode(msg.data)

    def send_mode(self, manual):
        packet = make_msg(*SET_MODE, struct.pack("<B", int(manual)))
        self.get_logger().info(f"setting {self.mode_name(manual)}: {hex_of(packet)}")
        try:
            os.write(self.fd, packet)
        except OSError as e:
            self.disconnect(e)

    def poll(self):
        if self.fd is None:
            self.connect()
            return
        try:
            data = os.read(self.fd, 1024)
        except BlockingIOError:
            return
        except OSError as e:
            self.disconnect(e)
            return
        if not data:
            self.disconnect("end of file")
            return
        found, self.buffer = packets(self.buffer + data)
        for class_id, subclass_id, payload, ok in found:
            self.get_logger().info(
                f"board sent class 0x{class_id:02x} subclass 0x{subclass_id:02x}"
                f" payload [{hex_of(payload)}]" + ("" if ok else " BAD CHECKSUM"),
            )
            if ok and payload and (class_id, subclass_id) == self.mode_packet:
                self.on_board_mode(payload[0] != 0)

    def on_board_mode(self, manual):
        if not self.report_mode:
            return
        if manual != self.reported:
            self.get_logger().info(f"board says {self.mode_name(manual)}")
        self.reported = manual
        self.mode_pub.publish(Bool(data=manual))

    @staticmethod
    def mode_name(manual):
        return "MANUAL" if manual else "AUTONOMOUS"

    def destroy_node(self):
        if self.fd is not None:
            os.close(self.fd)
        super().destroy_node()


def main():
    rclpy.init()
    node = ManualModeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
