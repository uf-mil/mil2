#!/usr/bin/env python3

import contextlib
import os
import pty
import unittest
from dataclasses import dataclass
from enum import Enum, IntEnum
from threading import Thread

import launch
import launch_testing.actions
import pytest
import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.utilities
from electrical_protocol import Packet
from launch_ros.actions import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty


@pytest.mark.launch_test
def generate_test_description():
    calculator_device = Node(
        package="electrical_protocol",
        executable="calculator",
        name="calculator_device",
    )

    return launch.LaunchDescription(
        [
            calculator_device,
            launch_testing.actions.ReadyToTest(),
        ],
    )


class CalculatorMode(IntEnum):
    ADD = 0
    SUB = 1
    MUL = 2


class Sign(Enum):
    PLUS = "+"
    MINUS = "-"
    MULTIPLY = "*"


@dataclass
class RequestAddPacket(Packet, class_id=0x37, subclass_id=0x00, payload_format="<ff"):
    number_one: float
    number_two: float


@dataclass
class RequestSubPacket(Packet, class_id=0x37, subclass_id=0x01, payload_format="<ff"):
    start: float
    minus: float


@dataclass
class AnswerPacket(Packet, class_id=0x37, subclass_id=0x02, payload_format="<f"):
    result: float


@dataclass
class CharacterPacket(Packet, class_id=0x37, subclass_id=0x03, payload_format="<c10s"):
    single_char: str
    big_str: str


@dataclass
class EnumPacket(Packet, class_id=0x37, subclass_id=0x04, payload_format="<cb"):
    symbol: Sign
    number: CalculatorMode


class TestSimulatedBasic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def __init__(self, *args):
        super().__init__(*args)

    def setUp(self):
        self.node = rclpy.create_node("test_simulated_basic")

        def executor_spin(executor):
            with contextlib.suppress(rclpy.executors.ExternalShutdownException):
                executor.spin()

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = Thread(
            target=executor_spin,
            args=(self.executor,),
            daemon=True,
        )
        self.executor_thread.start()
        self.reentrant_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.port_publisher = self.node.create_publisher(
            String,
            "/calculator_device/port",
            1,
        )
        self.answer_one_subscriber = self.node.create_subscription(
            Float32,
            "/calculator_device/answer_one",
            self.answer_callback_one,
            10,
        )
        self.answer_two_subscriber = self.node.create_subscription(
            Float32,
            "/calculator_device/answer_two",
            self.answer_callback_two,
            10,
        )
        self.count = 0

    def test_simulated(self):
        print("starting test...")
        self.master, self.slave = pty.openpty()
        serial_name = os.ttyname(self.slave)
        print(serial_name)
        while not self.port_publisher.get_subscription_count() and rclpy.utilities.ok():
            print("waiting for port connection...")
            self.node.get_clock().sleep_for(Duration(seconds=0.5))
        self.port_publisher.publish(String(data=serial_name))
        self.trigger_service_caller = self.node.create_client(
            Empty,
            "/calculator_device/trigger_one",
            callback_group=self.reentrant_cbg,
        )
        self.trigger_service_caller.wait_for_service()
        for i in range(1000):
            self.node.get_logger().info(f"i: {i}")
            # In ROS2, this has to be call_async -- call will not work, as it will
            # block the callback, blocking all other futures from running :(
            self.trigger_service_caller.call_async(Empty.Request())
            packet_bytes = os.read(self.master, 100)
            packet = RequestAddPacket.from_bytes(packet_bytes)
            self.assertEqual(
                packet.number_one,
                i,
                f"packet.number_one: {packet.number_one}, i: {i}",
            )
            self.assertEqual(
                packet.number_two,
                1000 - i,
                f"packet.number_two: {packet.number_two}, i: {i}",
            )
            os.write(
                self.master,
                bytes(AnswerPacket(packet.number_one + packet.number_two)),
            )
            self.node.get_logger().info(f"finished {i}")
            self.node.get_clock().sleep_for(Duration(seconds=0.005))
        self.node.get_clock().sleep_for(Duration(seconds=2))
        self.assertGreaterEqual(self.count, 900)
        self.trigger_two_service_caller = self.node.create_client(
            Empty,
            "/calculator_device/trigger_two",
        )
        self.trigger_two_service_caller.call_async(Empty.Request())
        packet_bytes = os.read(self.master, 100)
        packet = CharacterPacket.from_bytes(packet_bytes)
        self.assertEqual(
            packet.single_char,
            "a",
            f"packet.single_char: {packet.single_char}",
        )
        self.assertEqual(
            packet.big_str,
            "small",
            f"packet.big_str: {packet.big_str}",
        )
        os.write(
            self.master,
            bytes(EnumPacket(Sign.MULTIPLY, CalculatorMode.ADD)),
        )
        print("totally done!")

    def answer_callback_one(self, msg: Float32):
        # at least 900 packets gotten (sometimes lower due to performance)
        self.assertGreaterEqual(msg.data, 900)
        self.count += 1
        self.node.get_logger().info(f"finished count: {self.count}")

    def answer_callback_two(self, msg: Float32):
        self.assertEqual(msg.data, 5)

    def tearDown(self):
        os.close(self.master)
        os.close(self.slave)
        self.node.destroy_node()
        self.executor.shutdown()
        self.executor_thread.join()
