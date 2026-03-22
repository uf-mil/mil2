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

    def move(self, deltas: Coord4D):
        """
        Sends a relative movement goal to the action server.
        """
        req = Move.Goal()
        req.type = "Relative"

        dx, dy, dz, dyaw = deltas

        req.goal_pose.position.x = dx
        req.goal_pose.position.y = dy
        req.goal_pose.position.z = dz

        req.goal_pose.orientation.x = 0
        req.goal_pose.orientation.y = 0
        req.goal_pose.orientation.z = math.sin(dyaw / 2)
        req.goal_pose.orientation.w = math.cos(dyaw / 2)

        future = self.client.send_goal(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def _wait_for_service(self):
        while not self.client.wait_for_server():
            self.get_logger().info(
                "Waiting for /subjugator_localization/enable service...",
            )
