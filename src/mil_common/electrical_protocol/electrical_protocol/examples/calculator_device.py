#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from threading import Event
from typing import Union

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty

from electrical_protocol import Packet, SerialDeviceNode


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


class CalculatorDevice(
    SerialDeviceNode[
        Union[RequestAddPacket, RequestSubPacket, CharacterPacket],
        Union[AnswerPacket, EnumPacket],
    ],
):
    def __init__(self):
        super().__init__("calculator_device", None, 115200)
        self.port_topic = self.create_subscription(
            String,
            "~/port",
            self.port_callback,
            1,
        )
        self.start_one_service = self.create_service(
            Empty,
            "~/trigger_one",
            self.trigger,
        )
        self.start_two_service = self.create_service(
            Empty,
            "~/trigger_two",
            self.trigger_two,
        )
        self.answer_one_topic = self.create_publisher(Float32, "~/answer_one", 10)
        self.answer_two_topic = self.create_publisher(Float32, "~/answer_two", 10)
        self.next_packet = Event()
        self.i = 0

    def port_callback(self, msg: String):
        self.connect(msg.data, 115200)

    def trigger(self, _: Empty.Request, response: Empty.Response):
        self.num_one, self.num_two = self.i, 1000 - self.i
        self.i += 1
        self.send_packet(
            RequestAddPacket(number_one=self.num_one, number_two=self.num_two),
        )
        self.get_clock().sleep_for(Duration(seconds=1))
        return response

    def trigger_two(self, _: Empty.Request, response: Empty.Response):
        self.send_packet(CharacterPacket("a", "small"))
        return response

    def on_packet_received(self, packet) -> None:
        if isinstance(packet, AnswerPacket):
            self.answer_one_topic.publish(Float32(data=packet.result))
            self.next_packet.set()
        elif isinstance(packet, EnumPacket):
            self.answer_two_topic.publish(Float32(data=packet.number.value))


def main(args=None):
    rclpy.init(args=args)
    calculator_device = CalculatorDevice()
    rclpy.spin(calculator_device)
    rclpy.shutdown()


if __name__ == "__main__":
    main()