#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from electrical_protocol import AckPacket
from mil_usb_to_can.sub9 import CANDeviceHandle

from rclpy import node, service, timer
import rclpy.logging
import rclpy.logging_service
import rclpy.service
import rclpy.time
import rclpy.timer
import rclpy.node
from sub_actuator_board.srv import (
    GetValve,
    GetValveRequest,
    GetValveResponse,
    SetValve,
    SetValveRequest,
)

from .packets import (
    ActuatorPollRequestPacket,
    ActuatorPollResponsePacket,
    ActuatorSetPacket,
)

# TODO: make the parent class inherit from node so we can have access to the clock
class ActuatorBoard(CANDeviceHandle):
    """
    Device handle for the actuator board. Because this class implements a CAN device,
    it inherits from the :class:`CANDeviceHandle` class.
    """

    _recent_response: ActuatorPollResponsePacket | None

    def __init__(self, node:rclpy.node.Node, *args, **kwargs): # TODO: once parent class is established remove node from argument
        super().__init__(*args, **kwargs)
        self._set_service = rclpy.service.Service("/set_valve", SetValve, self.set_valve)
        self._get_service = rclpy.service.Service("/get_valve", GetValve, self.get_valve)
        self.observed_node = node
        self._recent_response = None

    def set_valve(self, req: SetValveRequest) -> dict:
        """
        Called when the ``/set_valve`` service is requested. Creates a message to
        control the valve and sends it through the inherited device handle.

        Args:
            req (SetValveRequest): Request to set the valve.

        Returns:
            dict: List of information which is casted into a SetValveResponse.
        """
        # Send board command to open or close specified valve
        message = ActuatorSetPacket(address=req.actuator, open=req.opened)
        self.send_data(message)

        self.observed_node.get_logger().info(
            "Set valve {} {}".format(
                req.actuator,
                "opened" if req.opened else "closed",
            ),
        )
        # Wait some time for board to process command
        rclpy.timer.Rate.sleep(0.01)
        # Request the status of the valve just commanded to ensure it worked
        self.send_data(ActuatorPollRequestPacket())

        #TODO: Time acquisition is done by node and not globally anymore
        start = self.observed_node.get_clock().now().to_msg().sec
        while self.observed_node.get_clock().now().to_msg().sec - start < 2:
            if self._recent_response is not None:
                break

        success = False
        if self._recent_response is not None:
            if not req.opened:
                success = not (self._recent_response.values & (1 << req.actuator))
            else:
                success = self._recent_response.values & (1 << req.actuator)
        response = {"success": success}
        self._recent_response = None
        return response

    def get_valve(self, req: GetValveRequest) -> GetValveResponse:
        message = ActuatorPollRequestPacket()
        self.send_data(message)
        start = self.observed_node.get_clock().now().to_msg().sec
        while self.observed_node.get_clock().now().to_msg().sec - start < 10:
            if self._recent_response is not None:
                break

        if not self._recent_response:
            raise RuntimeError("No response from the board within 10 seconds.")

        response = GetValveResponse(
            opened=self._recent_response.values & (1 << req.actuator),
        )
        self._recent_response = None
        return response

    def on_data(self, packet: ActuatorPollResponsePacket | AckPacket) -> None:
        """
        Process data received from board.
        """
        if isinstance(packet, ActuatorPollResponsePacket):
            self._recent_response = packet
