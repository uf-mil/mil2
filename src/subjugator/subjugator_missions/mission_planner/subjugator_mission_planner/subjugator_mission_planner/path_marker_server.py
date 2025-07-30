import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Move, PathMarker
from subjugator_msgs.msg import Orange
from tf_transformations import quaternion_from_euler

# lowk this assumes that the stupid thing is in frame

# this isn't DRY either...
IMAGE_WIDTH = 840
IMAGE_HEIGHT = 680


# TODO this is not DRY, use Adam's fix...
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


class PathMarkerServer(Node):
    def __init__(self):
        super().__init__("pathmarker")

        # Action server
        self._action_server = ActionServer(
            self,
            PathMarker,
            "pathmarker",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.orange_sub = self.create_subscription(
            Orange,
            "centroids/orange",
            self.recent_detection,
            10,
        )
        self.spotted = False
        self.last_orange = Orange()

        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.recent_odom,
            10,
        )
        self.last_odom = Odometry()

        self.move_client = ActionUser(self, Move, "move")

    def recent_odom(self, msg: Odometry):
        self.last_odom = msg

    def recent_detection(self, msg: Orange):
        self.spotted = True
        self.last_orange = msg

    # TODO this is also not DRY... :( :P
    def sleep_for(self, time: float):
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=time)

        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks while waiting

    def goal_callback(self, goal_request):
        self.wait_time = goal_request.time
        self.get_logger().info("haha path marker")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def control_loop(self) -> Pose:
        if not self.spotted:
            return Pose()

        x_error_pixels = IMAGE_WIDTH / 2 - self.last_orange.width
        kpx = -2.5 / (2 * IMAGE_WIDTH)

        y_error_pixels = IMAGE_HEIGHT / 2 - self.last_orange.height
        kpy = -2.5 / (2 * IMAGE_HEIGHT)

        print("---------")
        print("x error pixels: ", x_error_pixels)
        x_command = -x_error_pixels * kpx
        print("x command: ", x_command)

        print("y error pixels: ", y_error_pixels)
        y_command = -y_error_pixels * kpy
        print("y command: ", y_command)
        print("---------")

        # Create goal pose (relative to current pose)
        goal_pose = Pose()
        goal_pose.position.x = float(x_command)
        goal_pose.position.y = float(y_command)
        goal_pose.position.z = 0

        # Publish the goal pose
        self.spotted = False
        return goal_pose

    def execute_callback(self, goal_handle):
        # rotate while tracking the path marker
        # we want to keep it in frame while also rotating
        # yaw 360 degrees while also tracking the silly thing
        num_steps = 20
        max_height = -1
        best_pose = Pose()
        for _ in range(num_steps):
            yaw_rad = (2 * math.pi) / num_steps
            w, x, y, z = quaternion_from_euler(0, 0, yaw_rad)
            pose = self.control_loop()
            pose.orientation.x = x
            pose.orientation.y = y
            pose.orientation.z = z
            pose.orientation.w = w

            # move to goal
            goal = Move.Goal()
            goal.goal_pose = pose
            goal.type = "Relative"
            self.move_client.send_goal_and_block_until_done(goal)

            self.sleep_for(1.0)

            # check height
            if self.last_orange.height > max_height:
                # TODO this could totally use waypoints
                max_height = self.last_orange.height
                best_pose.position.x = self.last_odom.pose.pose.position.x
                best_pose.position.y = self.last_odom.pose.pose.position.y
                best_pose.position.z = self.last_odom.pose.pose.position.z
                best_pose.orientation.w = self.last_odom.pose.pose.orientation.w
                best_pose.orientation.x = self.last_odom.pose.pose.orientation.x
                best_pose.orientation.y = self.last_odom.pose.pose.orientation.y
                best_pose.orientation.z = self.last_odom.pose.pose.orientation.z

        # go to best pose
        goal = Move.Goal()
        goal.goal_pose = best_pose
        goal.type = "Absolute"
        self.move_client.send_goal_and_block_until_done(goal)

        # go forwards
        goal = Move.Goal()
        p = Pose()
        p.position.x = 1.0
        p.orientation.w = 1.0
        goal.goal_pose = p
        goal.type = "relative"
        self.move_client.send_goal_and_block_until_done(goal)

        goal_handle.succeed()
        result = PathMarker.Result()
        result.success = True
        result.message = "Wait complete"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PathMarkerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
