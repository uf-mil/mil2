#!/usr/bin/python3
from __future__ import annotations

import random

import rclpy
from rclpy.node import Node

from electrical_protocol import AckPacket
from mil_usb_to_can.sub9 import SimulatedCANDeviceHandle

from .packets import (
    ActuatorPollRequestPacket,
    ActuatorPollResponsePacket,
    ActuatorSetPacket,
)


class ActuatorBoardSimulation(SimulatedCANDeviceHandle, Node):
    """
    Simulator for the communication of the actuator board.

    Attributes:
        status (Dict[int, bool]): The status of the valves. The keys are each of the valve IDs,
          and the values are the statues of whether the valves are open.
    """

    def __init__(self, *args, **kwargs):
        # Tracks the status of the 12 valves
        self.status = {i: False for i in range(4)}
        super().__init__(*args, **kwargs)
        Node.__init__(self, "actuator_board_sim")

    def on_data(self, packet: ActuatorSetPacket | ActuatorPollRequestPacket) -> None:
        """
        Processes data received from motherboard / other devices. For each message received,
        the class' status attribute is updated if the message is asking to write, otherwise
        a feedback message is constructed and sent back.
        """
        # If message is writing a valve, store this change in the internal dictionary
        if isinstance(packet, ActuatorSetPacket):
            self.create_rate(random.randrange(0, 1)).sleep()  # Time to simluate opening of valves
            self.status[packet.address] = packet.open
            self.send_data(bytes(AckPacket()))

        # If message is a status request, send motherboard the status of the requested valve
        elif isinstance(packet, ActuatorPollRequestPacket):
            self.send_data(
                bytes(
                    ActuatorPollResponsePacket(
                        int(
                            "".join(
                                str(int(x)) for x in reversed(self.status.values())
                            ),
                            base=2,
                        ),
                    ),
                ),
            )