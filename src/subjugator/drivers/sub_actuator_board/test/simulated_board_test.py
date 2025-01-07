#!/usr/bin/env python3
import unittest
import asyncio

import rclpy
import rclpy.executors

from sub_actuator_board.packets import (
    ActuatorPacketId,
    ActuatorPollRequestPacket,
    ActuatorPollResponsePacket,
    ActuatorSetPacket,
)
from subjugator_msgs.srv import SetValve, GetValve


class SimulatedBoardTest(unittest.TestCase):
    """
    Integration test for CAN2USB board driver. Talks
    to a simulated CAN device which should add two integers.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("simulated_board_test")
        cls.set_srv = cls.node.create_client(SetValve, "/set_valve")
        cls.get_srv = cls.node.create_client(GetValve, "/get_valve")
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for_service(self, client, timeout=5.0):
        """Waits for a service to be available."""
        output = client.wait_for_service(timeout_sec=timeout)
        print(f"WAITING FOR SERVICE OUTPUT: {output}")
        return output

    async def call_service(self, client, request):
        """Calls a service and returns the response."""
        future = client.call_async(request)
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()

    def test_correct_response(self):
        """
        Test we can get correct data through CAN bus.
        """
        self.assertTrue(self.wait_for_service(self.set_srv))
        self.assertTrue(self.wait_for_service(self.get_srv))

        # Set and Get Valve State
        request = SetValve.Request(id=0, opened=True)
        response = asyncio.run(self.call_service(self.set_srv, request))
        self.assertTrue(response.success)

        request = GetValve.Request(id=0)
        response = asyncio.run(self.call_service(self.get_srv, request))
        self.assertTrue(response.opened)

        request = SetValve.Request(id=0, opened=False)
        response = asyncio.run(self.call_service(self.set_srv, request))
        self.assertTrue(response.success)

        request = GetValve.Request(id=0)
        response = asyncio.run(self.call_service(self.get_srv, request))
        self.assertFalse(response.opened)

    def test_packet(self):
        """
        Test that the bytes representation of all packets is correct.
        """
        # ActuatorPollRequestPacket
        packet = ActuatorPollRequestPacket()
        self.assertEqual(bytes(packet), b"7\x01\x03\x01\x00\x00\x04\x0f")
        # ActuatorPollResponsePacket
        packet = ActuatorPollResponsePacket(0b00111)
        self.assertEqual(packet.ball_drop_opened, True)
        self.assertEqual(packet.gripper_opened, True)
        self.assertEqual(packet.torpedo_launcher_opened, True)
        self.assertEqual(bytes(packet), b"7\x01\x03\x02\x01\x00\x07\r!")
        # ActuatorSetPacket
        packet = ActuatorSetPacket(ActuatorPacketId.TORPEDO_LAUNCHER, True)
        self.assertEqual(bytes(packet), b"7\x01\x03\x00\x02\x00\x01\x01\x07\x1d")


if __name__ == "__main__":
    unittest.main()
