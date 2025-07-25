import rclpy
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Waypoint
from subjugator_msgs.srv import StringTrigger


class WpServer(Node):
    def __init__(self):
        super().__init__("wp_server")

        # Action server
        self._action_server = ActionServer(
            self,
            Waypoint,
            "waypoint_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.set_client = self.create_client(StringTrigger, "wp/set")
        self.goto_client = self.create_client(StringTrigger, "wp/goto")

    def goal_callback(self, goal_request: Waypoint.Goal):
        self.wp_name = goal_request.wp_name
        self.wp_goto = goal_request.goto_wp
        self.wp_set = goal_request.set_wp
        self.get_logger().info(
            f"name: {self.wp_name}. goto: {self.wp_goto}. set: {self.wp_set}",
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        if self.wp_set and self.wp_goto:
            # can't do both so will crash
            goal_handle.self.fail("both set and goto were true!")
            result = Waypoint.Result()
            result.success = False
            result.message = "both set and goto were true!"
            return result

        if not (self.wp_set or self.wp_goto):
            # do nothing? screw you!
            goal_handle.self.fail("neither set what the heck")
            result = Waypoint.Result()
            result.success = False
            result.message = "neither set what the heck"
            return result

        # now just set or goto the waypoint
        if self.wp_set:
            req = StringTrigger.Request()
            req.wp_name = self.wp_name
            self.set_client.call(req)
        elif self.wp_goto:
            req = StringTrigger.Request()
            req.wp_name = self.wp_name
            self.goto_client.call(req)
        else:
            # unreachable
            pass

        goal_handle.succeed()
        result = Waypoint.Result()
        result.success = True
        result.message = "wp set" if self.wp_set else "wp reached"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WpServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
