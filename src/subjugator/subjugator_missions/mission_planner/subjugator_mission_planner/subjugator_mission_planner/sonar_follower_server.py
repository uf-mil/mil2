import math

import rclpy
from geometry_msgs.msg import Pose
from mil_msgs.msg import ProcessedPing
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Move, SonarFollower


class ActionUser:
    """
    class to make calling other missions easier, you do still kinda have to construct the goals on your own sry
    """

    def __init__(self, node: Node, action_type, action_name: str):
        self.node = node
        self.ac = ActionClient(
            node,
            action_type,
            action_name,
            callback_group=ReentrantCallbackGroup(),
        )

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

        self.freq = 999999

        # Action server
        self._action_server = ActionServer(
            self,
            SonarFollower,
            "sonarfollower",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
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
        if True:  # abs(msg.frequency - self.freq) < 2500: # TODO remove
            self.last_ping = msg
            self.heard_ping = True
        else:
            print("ignoring a ping with freq = ", msg.frequency)

    def goal_callback(self, goal_request):
        self.get_logger().info("goal to follow sonar")
        self.freq = goal_request.frequency
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def compare_pings(self, p1: ProcessedPing | None, p2: ProcessedPing):
        self.get_logger().warn(str(p2.origin_direction_body.z))
        return abs(p2.origin_direction_body.z) >= 0.75

        if p1 is None:
            return False

        # normalize both
        x1 = float(p1.origin_direction_body.x)
        y1 = float(p1.origin_direction_body.y)
        p1_mag = math.sqrt(x1 * x1 + y1 * y1)
        x1 = x1 / p1_mag
        y1 = y1 / p1_mag

        x2 = float(p2.origin_direction_body.x)
        y2 = float(p2.origin_direction_body.y)
        p2_mag = math.sqrt(x2 * x2 + y2 * y2)
        x2 = x2 / p2_mag
        y2 = y2 / p2_mag

        # dot product
        dot_product = (
            x1 * x2 + y1 * y2
        )  # this IS the cos of the angle between them (so unitless)

        # inverse cos
        dot_product = min(1.0, dot_product)
        dot_product = max(-1.0, dot_product)
        angle_rad = math.acos(dot_product)  # in radians!!
        angle_degrees = angle_rad * (180 / math.pi)
        print(angle_degrees)

        # check angle
        return angle_degrees > 100

    def execute_callback(self, goal_handle):
        passed_pinger = False
        previous_ping = None
        while not passed_pinger:
            while not self.heard_ping:
                self.get_logger().warn("no new pinger heard! sleeping")
                self.sleep_for(0.1)
            self.heard_ping = False  # this is lowk stupid TODO

            # check and see if the current ping is pointing the opposite direction from the past ping
            passed_pinger = self.compare_pings(previous_ping, self.last_ping)
            if passed_pinger:
                break

            previous_ping = self.last_ping

            # just move towards it no rotation
            x = self.last_ping.origin_direction_body.x
            y = self.last_ping.origin_direction_body.y
            _ = self.last_ping.origin_direction_body.z

            p = Pose()
            p.orientation.w = 1.0
            p.position.x = 0.5 * x
            p.position.y = 0.5 * y
            goal = Move.Goal()
            goal.type = "Relative"
            goal.goal_pose = p
            self.move_client.send_goal_and_block_until_done(goal)
            self.sleep_for(0.5)

        goal_handle.succeed()
        result = SonarFollower.Result()
        result.success = True
        result.message = "surfaced at sonar"
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
