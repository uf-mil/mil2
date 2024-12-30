import random
import string
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from example_interfaces.srv import AddTwoInts  # ROS 2 equivalent for AddTwoInts
from electrical_protocol import Packet

from .device import CANDeviceHandle, SimulatedCANDeviceHandle


@dataclass
class ExampleEchoRequestPacket(
    Packet,
    class_id=0x99,
    subclass_id=0x00,
    payload_format="<10s",
):
    my_special_string: bytes


@dataclass
class ExampleEchoResponsePacket(
    Packet,
    class_id=0x99,
    subclass_id=0x01,
    payload_format="<10s",
):
    my_special_string: bytes


@dataclass
class ExampleAdderRequestPacket(
    Packet,
    class_id=0x99,
    subclass_id=0x02,
    payload_format="<BB",
):
    num_one: int
    num_two: int


@dataclass
class ExampleAdderResponsePacket(
    Packet,
    class_id=0x99,
    subclass_id=0x03,
    payload_format="<B",
):
    response: int


class ExampleEchoDeviceHandle(CANDeviceHandle, Node):
    """
    An example implementation of a CANDeviceHandle which will handle
    a device that echoes back any data sent to it.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        Node.__init__(self, "echo_device")

        self.last_sent = None
        self.count = 0
        self._srv = self.create_service(Trigger, "start_echo", self.srv_req)

    def srv_req(self, req, response):
        while self.count < 10:
            if not self.send_new_string(10):
                response.success = False
                response.message = "Unable to send string of length ten."
                return response
        response.success = True
        response.message = "Complete!"
        return response

    def on_data(self, data: ExampleEchoRequestPacket):
        response = data.my_special_string.decode()
        if self.last_sent is None:
            raise RuntimeError(f"Received {data} but have not yet sent anything")
        elif response != self.last_sent[0]:
            raise ValueError(
                f"ERROR! Received {response} but last sent {self.last_sent}",
            )
        else:
            self.count += 1

    def send_new_string(self, length: int = 10):
        # Generate a test string
        test = "".join(random.choice(string.ascii_letters) for _ in range(length))
        self.last_sent = (test, self.get_clock().now())
        self.send_data(ExampleEchoRequestPacket(test.encode()))
        start = self.get_clock().now()
        count_now = self.count
        while self.count == count_now:
            if self.get_clock().now() - start > Duration(seconds=1):
                return False
        return True


class ExampleAdderDeviceHandle(CANDeviceHandle, Node):
    """
    An example implementation of a CANDeviceHandle which will handle
    a device that processes addition requests.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        Node.__init__(self, "adder_device")
        self.response_received = None
        self._srv = self.create_service(AddTwoInts, "add_two_ints", self.on_service_req)

    def on_service_req(self, req, response):
        self.response_received = None
        self.send_data(ExampleAdderRequestPacket(req.a, req.b))
        start = self.get_clock().now()
        while self.response_received is None:
            if self.get_clock().now() - start > Duration(seconds=1):
                response.sum = -1
                return response
        response.sum = self.response_received.response
        return response

    def on_data(self, data: ExampleAdderResponsePacket):
        self.response_received = data


class ExampleSimulatedEchoDevice(SimulatedCANDeviceHandle):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echoes this data back.
    """

    def __init__(self, handle, inbound_packets):
        super().__init__(handle, inbound_packets)

    def on_data(self, data: ExampleEchoRequestPacket):
        # Echo data received back onto the bus
        self.send_data(bytes(ExampleEchoResponsePacket(data.my_special_string)))


class ExampleSimulatedAdderDevice(SimulatedCANDeviceHandle):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echoes this data back.
    """

    def __init__(self, handle, inbound_packets):
        super().__init__(handle, inbound_packets)

    def on_data(self, data: ExampleAdderRequestPacket):
        self.send_data(bytes(ExampleAdderResponsePacket(data.num_one + data.num_two)))