import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String

from subjugator_mission_planner.action import NavigateThroughObject


class NavigateThroughObjectServer(Node):
    def __init__(self):
        super().__init__("navigate_through_object_server")

        self._action_server = ActionServer(
            self,
            NavigateThroughObject,
            "navigate_through_object",
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
        self.get_logger().info(
            f"Received nav-through goal: object={goal_request.object}",
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def perception_callback(self, msg):
        self.current_detection = msg.data

    async def execute_callback(self, goal_handle):
        requested_object = goal_handle.request.object
        self.get_logger().info(f"Starting navigation towards: {requested_object}")

        aligned = False

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateThroughObject.Result()
                result.success = False
                result.message = "Canceled"
                return result

            # ACTUAL CONTROL LOOP OF GOING THROUGH OBJECT
            # PROBABLY MOVE FORWARD TO IT IN SMALLER BITS INSTEAD OF ALIGNING AND FULL SENDING
            if self.current_detection == requested_object:
                if not aligned:
                    self.get_logger().info("Object detected: aligning...")

                    await asyncio.sleep(2)  # simulate alignment
                    aligned = True
                else:
                    self.get_logger().info("Moving through object...")
                    await asyncio.sleep(3)  # simulate passing through gate

                    self.get_logger().info("Successfully passed through!")
                    goal_handle.succeed()

                    result = NavigateThroughObject.Result()
                    result.success = True
                    result.message = "Passed through object"
                    return result

            else:
                self.get_logger().info("Waiting for object to appear...")
                await asyncio.sleep(0.5)
