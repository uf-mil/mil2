from __future__ import annotations

import abc
import contextlib
import threading
from typing import Any, Generic, TypeVar, Union, cast, get_args, get_origin

import serial
from rclpy.node import Node

from .packet import SYNC_CHAR_1, Packet

SendPackets = TypeVar("SendPackets", bound=Packet)
RecvPackets = TypeVar("RecvPackets", bound=Packet)


class SerialDeviceNode(Generic[SendPackets, RecvPackets], Node):
    """
    Represents a node capable of communicating with a serial device.

    Attributes:
        port (Optional[str]): The port used for the serial connection, if provided.
        baudrate (Optional[int]): The baudrate to use with the device, if provided.
        device (Optional[serial.Serial]): The serial class used to communicate with
            the device.
        rate (float): The reading rate of the device, in Hertz. Set to `20` by default.
    """

    device: serial.Serial | None
    _recv_T: Any
    _send_T: Any

    def is_connected(self) -> bool:
        return self.device is not None

    def is_open(self) -> bool:
        return bool(self.device) and self.device.is_open

    def __init__(self, node_name: str, port: str | None, baudrate: int | None) -> None:
        """
        Arguments:
            port (Optional[str]): The serial port to connect to. If ``None``, connection
                will not be established on initialization; rather, the user can use
                :meth:`~.connect` to connect later.
            baudrate (Optional[int]): The baudrate to connect with. If ``None`` and
                a port is specified, then 115200 is assumed.
        """
        super().__init__(node_name)
        self.port = port
        self.baudrate = baudrate
        if port:
            self.device = serial.Serial(port, baudrate or 115200, timeout=0.1)
            if not self.device.is_open:
                self.device.open()
            self.device.reset_input_buffer()
            self.device.reset_output_buffer()
        else:
            self.device = None
        self.lock = threading.Lock()
        self.rate = 20.0
        self.timer = self.create_timer(1.0 / self.rate, self._process_buffer)  # type: ignore

    def __init_subclass__(cls) -> None:
        # this is a super hack lol :P
        # cred: https://stackoverflow.com/a/71720366
        cls._send_T = get_args(cls.__orig_bases__[0])[0]  # type: ignore
        cls._recv_T = get_args(cls.__orig_bases__[0])[1]  # type: ignore

    def connect(self, port: str, baudrate: int) -> None:
        """
        Connects to the port with the given baudrate. If the device is already
        connected, the input and output buffers will be flushed.

        Arguments:
            port (str): The serial port to connect to.
            baudrate (int): The baudrate to connect with.
        """
        self.port = port
        self.baudrate = baudrate
        self.device = serial.Serial(port, baudrate, timeout=0.1)
        if not self.device.is_open:
            self.device.open()
        self.device.reset_input_buffer()
        self.device.reset_output_buffer()

    def close(self) -> None:
        """
        Closes the serial device.
        """
        self.get_logger().info("Closing serial device...")
        if not self.device:
            raise RuntimeError("Device is not connected.")
        else:
            # TODO: Find a better way to deal with these os errors
            with contextlib.suppress(OSError):
                if self.device.in_waiting:
                    self.get_logger().warning(
                        "Shutting down device, but packets were left in buffer...",
                    )
        self.device.close()

    def write(self, data: bytes) -> None:
        """
        Writes a series of raw bytes to the device. This method should rarely be
        used; using :meth:`~.send_packet` is preferred because of the guarantees
        it provides through the packet class.

        Arguments:
            data (bytes): The data to send.
        """
        if not self.device:
            raise RuntimeError("Device is not connected.")
        self.device.write(data)

    def send_packet(self, packet: SendPackets) -> None:
        """
        Sends a given packet to the device.

        Arguments:
            packet (:class:`~.Packet`): The packet to send.
        """
        with self.lock:
            self.write(bytes(packet))

    def _read_from_stream(self) -> bytes:
        # Read until SOF is encourntered in case buffer contains the end of a previous packet
        if not self.device:
            raise RuntimeError("Device is not connected.")
        sof = None
        for _ in range(10):
            sof = self.device.read(1)
            if not len(sof):
                continue
            sof_int = int.from_bytes(sof, byteorder="big")
            if sof_int == SYNC_CHAR_1:
                break
        if not isinstance(sof, bytes):
            raise TimeoutError("No SOF received in one second.")
        sof_int = int.from_bytes(sof, byteorder="big")
        if sof_int != SYNC_CHAR_1:
            self.get_logger().error("Where da start char at?")
        data = sof
        # Read sync char 2, msg ID, subclass ID
        data += self.device.read(3)
        length = self.device.read(2)  # read payload length
        data += length
        data += self.device.read(
            int.from_bytes(length, byteorder="little") + 2,
        )  # read data and checksum
        return data

    def _correct_type(self, provided: Any) -> bool:
        # either:
        #   1. RecvPackets is a Packet --> check isinstance on the type var
        #   2. RecvPackets is a Union of Packets --> check isinstance on all
        if get_origin(self._recv_T) is Union:
            return isinstance(provided, get_args(self._recv_T))
        else:
            return isinstance(provided, self._recv_T)

    def adjust_read_rate(self, rate: float) -> None:
        """
        Sets the reading rate to a specified amount.

        Arguments:
            rate (float): The reading speed to use, in hz.
        """
        self.rate = min(rate, 1_000)
        self.timer.timer_period_ns = int(1_000_000_000 / self.rate)

    def scale_read_rate(self, scale: float) -> None:
        """
        Scales the reading rate of the device handle by some factor.

        Arguments:
            scale (float): The amount to scale the reading rate by.
        """
        self.adjust_read_rate(self.rate * scale)

    def _read_packet(self) -> bool:
        if not self.device:
            raise RuntimeError("Device is not connected.")
        try:
            with self.lock:
                if not self.is_open() or self.device.in_waiting == 0:
                    return False
                if self.device.in_waiting > 200:
                    self.get_logger().warning(
                        "Packets are coming in much quicker than expected, upping rate...",
                        throttle_duration_sec=0.5,
                    )
                    self.scale_read_rate(1 + self.device.in_waiting / 1000)
            packed_packet = self._read_from_stream()
            assert isinstance(packed_packet, bytes)
            packet = Packet.from_bytes(packed_packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Error reading packet: {e}")
            return False
        except OSError:
            self.get_logger().error(
                "Cannot read from serial device.",
                throttle_duration_sec=1,
            )
            return False
        if not self._correct_type(packet):
            self.get_logger().error(
                f"Received unexpected packet: {packet} (expected: {self._recv_T})",
            )
            return False
        packet = cast(RecvPackets, packet)
        self.on_packet_received(packet)
        return True

    def _process_buffer(self) -> None:
        if not self.is_open():
            return
        try:
            self._read_packet()
        except Exception as e:
            self.get_logger().error(f"Error reading recent packet: {e}")
            import traceback

            traceback.print_exc()

    @abc.abstractmethod
    def on_packet_received(self, packet: RecvPackets) -> None:
        """
        Abstract method to be implemented by subclasses for handling packets
        sent by the physical electrical board.

        Arguments:
            packet (:class:`~.Packet`): The packet that is received.
        """
        pass
