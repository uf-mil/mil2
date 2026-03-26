import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from subjugator_msgs.action import Move

from mil_robogym.data_collection.types import Coord4D


class MoveClient(Node):
    """
    Handles starting and stopping PID controls.
    """

    def __init__(self):

        super().__init__("move_action_client")

        self.client = ActionClient(self, Move, "/move")

        self._wait_for_service()

    def move(self, deltas: Coord4D, move_type: str = "Relative") -> bool:
        """
        Sends a relative movement goal to the action server.
        """
        req = Move.Goal()
        req.type = move_type

        dx, dy, dz, dyaw = deltas

        req.goal_pose.position.x = float(dx)
        req.goal_pose.position.y = float(dy)
        req.goal_pose.position.z = 0.0  # float(dz)

        req.goal_pose.orientation.x = 0.0
        req.goal_pose.orientation.y = 0.0
        req.goal_pose.orientation.z = float(math.sin(dyaw / 2))
        req.goal_pose.orientation.w = float(math.cos(dyaw / 2))

        self.get_logger().info("Sending Request")

        future = self.client.send_goal_async(req)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Received Response")

        return result_future.result().result.success

    def _wait_for_service(self):
        while not self.client.wait_for_server():
            self.get_logger().info(
                "Waiting for /subjugator_localization/enable service...",
            )
