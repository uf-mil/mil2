#!/usr/bin/env python3
from __future__ import annotations

from electrical_protocol import AckPacket

from mil_usb_to_can.sub9.device import CANDeviceHandle

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


from subjugator_msgs.srv import GetValve, SetValve

from .packets import (
    ActuatorPollRequestPacket,
    ActuatorPollResponsePacket,
    ActuatorSetPacket,
)

# TODO: make the parent class inherit from node so we can have access to the clock

class ActuatorBoard(CANDeviceHandle, Node):
    """
    Device handle for the actuator board. Because this class implements a CAN device,
    it inherits from the :class:`CANDeviceHandle` class.
    """

    _recent_response: ActuatorPollResponsePacket | None

    def __init__(self, *args, **kwargs): # TODO: once parent class is established remove node from argument
        super().__init__(*args, **kwargs)
        Node.__init__(self, "actuator_board")

        self._set_service = self.create_service(SetValve, "/set_valve", self.set_valve)
        self._get_service = self.create_service(GetValve, "/get_valve", self.get_valve)
        self._recent_response = None

    def set_valve(self, req: SetValve) -> dict:
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

        self.get_logger().info(
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
        start = self.get_clock().now()
        while self.get_clock().now() - start < Duration(seconds=2):
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

    def get_valve(self, req: GetValve) -> GetValve:
        message = ActuatorPollRequestPacket()
        self.send_data(message)
        start = self.node.get_clock().now()
        while self.node.get_clock().now() - start < Duration(seconds=10):
            if self._recent_response is not None:
                break

        if not self._recent_response:
            raise RuntimeError("No response from the board within 10 seconds.")

        response = GetValve(
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

def main(args=None):
    rclpy.init(args=args)
    actuator_board = ActuatorBoard()
    rclpy.spin(actuator_board)
    rclpy.shutdown()

if __name__ == '__main__':
    main()