import rclpy
from geometry_msgs.msg import Pose
from mil_msgs.msg import ProcessedPing
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Move, SonarFollower


class ActionUser:
    """
    class to make calling other missions easier, you do still kinda have to construct the goals on your own sry
    """

    def __init__(self, node: Node, action_type, action_name: str):
        self.node = node
        self.ac = ActionClient(node, action_type, action_name)

    # 0 timeout seconds implies no timeout
    # TODO rn there is nothing for timeout_sec, would be a great first issue for someone :) (use self.node.get_clock().now())
    # _=0 should be timeout_sec = 0
    def send_goal_and_block_until_done(self, goal, _=0):
        future = self.ac.send_goal_async(goal)  # rn no feedback
        running = True
        while running:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if future.done():
                result = future.result()
                result_future = result.get_result_async()

                while not result_future.done():  # TODO this could be better
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                running = False

    def send_goal_and_return_future(self, goal):
        return self.ac.send_goal_async(goal)


class SonarFollowerNode(Node):
    def __init__(self):
        super().__init__("sonarfollower")

        # Action server
        self._action_server = ActionServer(
            self,
            SonarFollower,
            "sonarfollower",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.move_client = ActionUser(self, Move, "move")

        self.ping_sub = self.create_subscription(
            ProcessedPing,
            "hydrophones/solved",
            self.ping_cb,
            10,
        )
        self.last_ping = ProcessedPing()
        self.heard_ping = False

    def sleep_for(self, time: float):
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=time)

        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks while waiting

    def ping_cb(self, msg: ProcessedPing):
        self.last_ping = msg
        self.heard_ping = True

    def goal_callback(self, goal_request):
        self.get_logger().info("goal to follow sonar")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        while not self.heard_ping:
            self.sleep_for(0.5)
        self.heard_ping = False  # this is lowk stupid TODO

        for _ in range(5):

            # just move towards it no rotation
            x = self.last_ping.origin_direction_body.x
            y = self.last_ping.origin_direction_body.y
            _ = self.last_ping.origin_direction_body.z

            p = Pose()
            p.orientation.w = 1.0
            p.position.x = x
            p.position.y = y
            goal = Move.Goal()
            goal.type = "Relative"
            goal.goal_pose = p
            self.move_client.send_goal_and_block_until_done(goal)

        goal_handle.succeed()
        result = SonarFollower.Result()
        result.success = True
        result.message = "Wait complete"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SonarFollowerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
