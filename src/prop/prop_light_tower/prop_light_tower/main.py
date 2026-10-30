#!/usr/bin/env python3

"""Code meant to connect to servo tube and do test with"""

import struct

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

CLASS_ID = 0x10


def fletcher16(data: bytes):
    """Compute the Fletcher-16 checksum of a byte sequence."""
    sum1 = 0
    sum2 = 0
    for index, byte in enumerate(data):
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
        # print(f"Data {index}: {byte:#02x}")
        # print(f"sum1: {sum1}")
        # print(f"sum2: {sum2}")

    return (sum2 << 8) | sum1


def make_msg(subclass_id: int, payload: bytes) -> bytes:
    """
    Create a message packet to send via usb.

    Inputs:
    - class_id (int): the device that it is being sent to (check uf-mil.github.io page or USB-TO-CAN.H)
    - subclass_id (int): what operation is the device doing (check the github.io or .h file)
    - playload (byte): message being sent (again check the github.io or .h file)

    Returns
    - bytes: full message byte array
    """
    sync = b"\x37\x01"
    cid_byte = struct.pack("<B", CLASS_ID)
    scid_byte = struct.pack("<B", subclass_id)
    length = struct.pack("<H", len(payload))

    # get the current msg to find checksum
    data1 = cid_byte + scid_byte + length + payload

    checksum = fletcher16(data1)
    checksum_bytes = struct.pack("<H", checksum)

    data = sync + data1 + checksum_bytes

    return data


def msg_to_string(msg):
    """Convert a message to a string to print."""
    return " ".join(f"{byte:02x}" for byte in msg)


# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #

# Connect to device
# ON WINDOWS, CHANGE THIS TO COM# , /dev/ is chad linux
dev = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

# pre-declared variables cause I am lazy

# -------------------------------- subclasses -------------------------------- #
"""
RF Kill Controller
Subclasses:
    - 0x00 : Kill Set
    - 0x01 : Kill Received (DO NOT USE)
    - 0x02 : Set move
"""


def manual_cb(msg):
    """
    payload = struct.pack("<?B", True, 0) # kill
    payload = struct.pack("<?B", False, 0) # unkill
    msg = make_msg(0x00, payload)
    """
    print("Manual:", msg.data)
    payload = struct.pack("<B", msg.data)
    msg = make_msg(0x02, payload)
    dev.write(msg)

    dev.read(1000)
    # print(f"Received the message {msg_to_string(response)}")


rclpy.init()
node = Node("manual_control_receiver")
qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub = node.create_subscription(Bool, "/manual_control", manual_cb, qos)
rclpy.spin(node)
rclpy.shutdown()
