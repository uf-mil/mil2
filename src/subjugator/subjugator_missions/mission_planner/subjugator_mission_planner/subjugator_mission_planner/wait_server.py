import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Wait


class WaitServer(Node):
    def __init__(self):
        super().__init__("wait_server")

        # Action server
        self._action_server = ActionServer(
            self,
            Wait,
            "wait",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        self.wait_time = goal_request.time
        self.get_logger().info(f"Received goal to wait for {self.wait_time} seconds")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.wait_time))
        self.get_logger().info("Timer complete!")

        goal_handle.succeed()
        result = Wait.Result()
        result.success = True
        result.message = "Wait complete"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WaitServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
