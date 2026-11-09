#!/usr/bin/env python3

"""
ROS2 node for the RF Kill Controller (light tower) over USB.

- Subscribes to /manual_control (std_msgs/Bool) and sends the SETMOVE
  command to the board (unchanged from the original node).
- Continuously reads the serial port, decodes MIL electrical protocol
  packets, and tracks KILLREC messages from the board.
- Publishes the light tower color as a prop state on /prop/state
  (std_msgs/String: "KILLED", "MANUAL" or "AUTO") whenever it changes.

Light tower logic mirrors the firmware (rf-kill-project.c, get_light_state):
    killed          -> RED    -> "KILLED"
    manual mode     -> YELLOW -> "MANUAL"
    autonomous mode -> GREEN  -> "AUTO"
"""

import struct

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------------- #
#                                   PROTOCOL                                   #
# ---------------------------------------------------------------------------- #

SYNC = b"\x37\x01"
HEADER_LEN = 6  # sync(2) + class(1) + subclass(1) + length(2)
CHECKSUM_LEN = 2
MAX_PAYLOAD_SIZE = 256  # matches MIL_electrical_protocol_msg.h

META_CLASSID = 0x00
META_SUBID_NAKPACK = 0x00
META_SUBID_ACKPACK = 0x01

NAVRF_CLASSID = 0x10
NAVRF_SUBID_KILLSET = 0x00
NAVRF_SUBID_KILLREC = 0x01
NAVRF_SUBID_SETMOVE = 0x02

# navrf_killstatus_t
KILL_REASONS = {
    0: "MOBO_REQUESTED",
    1: "RF_KILL",
    2: "EMERGENCY_BUTTON_PRESSED",
    3: "UNKILLED",
}

STATE_KILLED = "KILLED"
STATE_MANUAL = "MANUAL"
STATE_AUTO = "AUTO"

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
    cid_byte = struct.pack("<B", class_id)
    scid_byte = struct.pack("<B", subclass_id)
    length = struct.pack("<H", len(payload))

    data1 = cid_byte + scid_byte + length + payload
    checksum = fletcher16(data1)
    checksum_bytes = struct.pack("<H", checksum)

    return SYNC + data1 + checksum_bytes


def msg_to_string(msg: bytes) -> str:
    """Convert a message to a hex string for logging."""
    return " ".join(f"{byte:02x}" for byte in msg)


class PacketParser:
    """
    Incremental parser for MIL electrical protocol packets.

    Feed it raw serial bytes; it returns every complete, checksum-valid
    packet as (class_id, subclass_id, payload, raw_bytes). Partial packets
    are kept until the rest arrives, and garbage is skipped by resyncing
    on the next sync word.
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes) -> list:
        self.buf.extend(data)
        packets = []

        while True:
            start = self.buf.find(SYNC)
            if start < 0:
                # keep a trailing 0x37 in case the 0x01 hasn't arrived yet
                if self.buf.endswith(SYNC[:1]):
                    del self.buf[:-1]
                else:
                    self.buf.clear()
                break

            if start > 0:
                del self.buf[:start]

            if len(self.buf) < HEADER_LEN:
                break

            class_id = self.buf[2]
            subclass_id = self.buf[3]
            (length,) = struct.unpack_from("<H", self.buf, 4)

            if length > MAX_PAYLOAD_SIZE:
                # not a real header, drop this sync byte and resync
                del self.buf[:1]
                continue

            total = HEADER_LEN + length + CHECKSUM_LEN
            if len(self.buf) < total:
                break

            raw = bytes(self.buf[:total])
            (checksum,) = struct.unpack_from("<H", raw, HEADER_LEN + length)

            if fletcher16(raw[2 : HEADER_LEN + length]) != checksum:
                del self.buf[:1]
                continue

            payload = raw[HEADER_LEN : HEADER_LEN + length]
            packets.append((class_id, subclass_id, payload, raw))
            del self.buf[:total]

        return packets


# ---------------------------------------------------------------------------- #
#                                    NODE                                      #
# ---------------------------------------------------------------------------- #


class ManualModeNode(Node):

    CLASS_ID = NAVRF_CLASSID
    SUBCLASS_ID = NAVRF_SUBID_SETMOVE  # Set move / manual mode subclass

    SERIAL_POLL_PERIOD_S = 0.01  # how often to drain the serial port
    RESPONSE_TIMEOUT_S = 1.0  # same window the old blocking read used

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
            # timeout=0 -> non-blocking reads; the poll timer drains the port
            self.ser = serial.Serial(port, baud, timeout=0)
            self.get_logger().info(f"Connected to serial port: {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            raise

        self.parser = PacketParser()

        # ---- Board state ---------------------------------------------------- #
        # The board boots killed (red) and in manual mode (yellow) until told
        # otherwise, so start from the same assumptions.
        self.killed = True
        self.is_manual = True
        self.published_state = None

        # Time of the last command written, used to warn if no ACK/NAK comes back
        self.awaiting_response_since = None

        # ---- ROS interfaces ------------------------------------------------- #
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.subscription = self.create_subscription(
            Bool,
            "/manual_control",
            self.manual_control_callback,
            qos,
        )

        # Latched so late subscribers still get the current state
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/prop/state", state_qos)

        self.serial_timer = self.create_timer(self.SERIAL_POLL_PERIOD_S, self.serial_poll)

        self.publish_state_if_changed()

        self.get_logger().info(
            "ManualModeNode started, listening on /manual_control, publishing /prop/state",
        )

    # ------------------------------------------------------------------------ #
    #                                 SENDING                                  #
    # ------------------------------------------------------------------------ #

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
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return

        self.awaiting_response_since = self.get_clock().now()

        # The board switches its alive color as soon as it accepts this command
        self.is_manual = bool(is_manual)
        self.publish_state_if_changed()

    # ------------------------------------------------------------------------ #
    #                                 RECEIVING                                #
    # ------------------------------------------------------------------------ #

    def serial_poll(self):
        """Drain whatever bytes are waiting and handle complete packets."""
        try:
            waiting = self.ser.in_waiting
            data = self.ser.read(waiting) if waiting else b""
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read failed: {e}", throttle_duration_sec=5.0)
            return

        if data:
            for class_id, subclass_id, payload, raw in self.parser.feed(data):
                self.handle_packet(class_id, subclass_id, payload, raw)

        self.check_response_timeout()

    def handle_packet(self, class_id: int, subclass_id: int, payload: bytes, raw: bytes):
        """Dispatch one decoded packet from the board."""
        if class_id == META_CLASSID:
            self.awaiting_response_since = None
            if subclass_id == META_SUBID_ACKPACK:
                self.get_logger().info(f"Response (ACK): {msg_to_string(raw)}")
            elif subclass_id == META_SUBID_NAKPACK:
                self.get_logger().warn(f"Response (NAK): {msg_to_string(raw)}")
            else:
                self.get_logger().info(f"Response: {msg_to_string(raw)}")
            return

        if class_id == NAVRF_CLASSID and subclass_id == NAVRF_SUBID_KILLREC:
            if len(payload) != 2:
                self.get_logger().warn(f"Malformed KILLREC: {msg_to_string(raw)}")
                return

            killed = payload[0] != 0
            reason = KILL_REASONS.get(payload[1], f"UNKNOWN({payload[1]})")
            self.get_logger().info(
                f"Kill status from board: {'KILLED' if killed else 'UNKILLED'} "
                f"(reason: {reason}) | Packet: {msg_to_string(raw)}",
            )

            self.killed = killed
            self.publish_state_if_changed()
            return

        self.get_logger().info(f"Unhandled packet: {msg_to_string(raw)}")

    def check_response_timeout(self):
        """Warn if a command got no ACK/NAK, like the old blocking read did."""
        if self.awaiting_response_since is None:
            return

        elapsed = (self.get_clock().now() - self.awaiting_response_since).nanoseconds / 1e9
        if elapsed >= self.RESPONSE_TIMEOUT_S:
            self.get_logger().warn("No response received from device")
            self.awaiting_response_since = None

    # ------------------------------------------------------------------------ #
    #                               PROP STATE                                 #
    # ------------------------------------------------------------------------ #

    def current_state(self) -> str:
        """Mirror the firmware's light logic: red if killed, else mode color."""
        if self.killed:
            return STATE_KILLED
        return STATE_MANUAL if self.is_manual else STATE_AUTO

    def publish_state_if_changed(self):
        state = self.current_state()
        if state == self.published_state:
            return

        self.state_pub.publish(String(data=state))
        self.get_logger().info(f"Prop state: {self.published_state} -> {state}")
        self.published_state = state

    # ------------------------------------------------------------------------ #
    #                                 CLEANUP                                  #
    # ------------------------------------------------------------------------ #

    def destroy_node(self):
        """Clean up serial connection on shutdown."""
        if self.ser.is_open:
            self.ser.reset_input_buffer()  # drain any remaining messages
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
