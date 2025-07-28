import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Mechanism
from subjugator_msgs.srv import Servo


class MechanismServer(Node):
    def __init__(self):
        super().__init__("mechanism_server")

        # Action server
        self._action_server = ActionServer(
            self,
            Mechanism,
            "mechanism",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.dropper_client = self.create_client(Servo, "dropper")
        self.torpedo_client = self.create_client(Servo, "torpedo")
        self.gripper_client = self.create_client(Servo, "gripper")

    def goal_callback(self, goal_request):
        self.mechanism = goal_request.mechanism
        self.angle = goal_request.angle
        self.get_logger().info(
            f"Received goal to move {self.mechanism} to {self.angle} degrees",
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        msg = Servo.Request()
        msg.angle = self.angle
        if self.mechanism == "dropper":
            self.dropper_client.call(msg)
        elif self.mechanism == "gripper":
            self.gripper_client.call(msg)
        elif self.mechanism == "torpedo":
            self.torpedo_client.call(msg)

        goal_handle.succeed()
        result = Mechanism.Result()
        result.success = True
        result.message = "Mechanism move complete"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MechanismServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
