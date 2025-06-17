import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String

from subjugator_mission_planner.action import SearchObject


class SearchObjectServer(Node):
    def __init__(self):
        super().__init__("search_object_server")

        self._action_server = ActionServer(
            self,
            SearchObject,
            "search_for_object",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.subscription = self.create_subscription(
            String,
            "/detected_objects",
            self.perception_callback,
            10,
        )

        self.current_detection = None

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received search goal: object={goal_request.object}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def perception_callback(self, msg):
        self.current_detection = msg.data

    async def execute_callback(self, goal_handle):
        requested_object = goal_handle.request.object

        self.get_logger().info(f"Searching for: {requested_object}")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return SearchObject.Result(success=False, detected_object="")

            if self.current_detection == requested_object:
                self.get_logger().info(f"Found target: {requested_object}")
                goal_handle.succeed()
                result = SearchObject.Result()
                result.success = True
                result.detected_object = requested_object
                return result

            await asyncio.sleep(0.1)
