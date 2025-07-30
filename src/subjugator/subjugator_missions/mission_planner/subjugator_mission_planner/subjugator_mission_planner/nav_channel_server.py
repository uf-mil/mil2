import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Move, NavChannel, YawTracker
from tf_transformations import euler_from_quaternion
from yolo_msgs.msg import Detection


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


# algo: spot red pole and draw line
# move right and


# TODO
# 1. this
# 2. pose relative
# 3. call other missions !?!!?


def closest_points_between_lines(a1, v1, b1, v2):
    """
    Finds the closest points on two lines defined by:
    - Line 1: a1 + t * v1
    - Line 2: b1 + s * v2

    Returns:
        p1_closest: closest point on line 1
        p2_closest: closest point on line 2
        midpoint: the average of the two (midpoint between the lines)
    """
    # Ensure numpy arrays
    a1 = np.array(a1, dtype=np.float64)
    v1 = np.array(v1, dtype=np.float64)
    b1 = np.array(b1, dtype=np.float64)
    v2 = np.array(v2, dtype=np.float64)

    # Define some dot products
    r = a1 - b1
    v1_dot_v1 = np.dot(v1, v1)
    v2_dot_v2 = np.dot(v2, v2)
    v1_dot_v2 = np.dot(v1, v2)
    v1_dot_r = np.dot(v1, r)
    v2_dot_r = np.dot(v2, r)

    denom = v1_dot_v1 * v2_dot_v2 - v1_dot_v2**2

    # If denom is zero, lines are parallel — handle gracefully
    if np.isclose(denom, 0.0):
        # Pick arbitrary point on line 1, project onto line 2
        t = 0
        s = v2_dot_r / v2_dot_v2
    else:
        t = (v1_dot_v2 * v2_dot_r - v2_dot_v2 * v1_dot_r) / denom
        s = (v1_dot_v2 * t + v2_dot_r) / v2_dot_v2

    # Closest points
    p1_closest = a1 + t * v1
    p2_closest = b1 + s * v2
    midpoint = (p1_closest + p2_closest) / 2.0

    return midpoint


class NavChannelServer(Node):
    def __init__(self):
        super().__init__("navchannel")

        # odom and image data
        self._odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_cb,
            10,
        )
        self.recent_odom: Odometry = Odometry()

        self.centroid_sub_ = self.create_subscription(
            Detection,
            "centroids/red_pole",
            self.red_pole_cb,
            10,
        )
        self.recent_detection: Detection = Detection()
        self.detection_count = 0

        # Action server
        self._action_server = ActionServer(
            self,
            NavChannel,
            "navchannel",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.move_client = ActionUser(self, Move, "move")
        self.yaw_tracker_client = ActionUser(self, YawTracker, "yawtracker")

    def odom_cb(self, msg: Odometry):
        self.recent_odom = msg

    def red_pole_cb(self, msg: Detection):
        self.recent_detection = msg
        self.detection_count += 1

    def goal_callback(self, goal_request: NavChannel.Goal):
        self.number_of_red_poles = goal_request.number_of_red_poles
        self.detection_count = 0
        self.get_logger().info(
            f"Received goal saying there are {self.number_of_red_poles} red_poles",
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def spot_and_center_red_pole(self):
        # p represents a 10 degree positive yaw
        p = Pose()
        p.orientation.z = 0.0872
        p.orientation.w = 0.9962

        self.detection_count = 0

        # turn left until you see a red pole:
        goal = Move.Goal()
        goal.type = "Relative"
        goal.goal_pose = p
        while self.detection_count < 5:
            self.move_client.send_goal_and_block_until_done(goal)
            self.sleep_for(0.5)

        # now that we see a red pole, we should center on it??
        # goal = YawTracker.Goal()
        # goal.topic_name = "centroids/red_pole"
        # self.yaw_tracker_client.send_goal_and_block_until_done(goal)

    def sleep_for(self, time: float):
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=time)

        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks while waiting

    def try_find_red_pole(self, p1: Odometry, p2: Odometry):
        pose1 = [
            p1.pose.pose.position.x,
            p1.pose.pose.position.y,
            p1.pose.pose.position.z,
        ]

        # Get Euler angles (roll, pitch, yaw)
        euler_angles1 = euler_from_quaternion(
            [
                p1.pose.pose.orientation.x,
                p1.pose.pose.orientation.y,
                p1.pose.pose.orientation.z,
                p1.pose.pose.orientation.w,
            ],
        )

        # Convert yaw angle to direction vector (x,y,z)
        yaw1 = euler_angles1[2]  # The third element is yaw
        dir1 = np.array([np.cos(yaw1), np.sin(yaw1), 0.0])

        pose2 = [
            p2.pose.pose.position.x,
            p2.pose.pose.position.y,
            p2.pose.pose.position.z,
        ]

        # Get Euler angles (roll, pitch, yaw)
        euler_angles2 = euler_from_quaternion(
            [
                p2.pose.pose.orientation.x,
                p2.pose.pose.orientation.y,
                p2.pose.pose.orientation.z,
                p2.pose.pose.orientation.w,
            ],
        )

        # Convert yaw angle to direction vector (x,y,z)
        yaw2 = euler_angles2[2]  # The third element is yaw
        dir2 = np.array([np.cos(yaw2), np.sin(yaw2), 0.0])

        return closest_points_between_lines(pose1, dir1, pose2, dir2)

    def execute_callback(self, goal_handle):
        self.get_logger().warn("spot 1")
        self.spot_and_center_red_pole()

        goal = YawTracker.Goal()
        goal.topic_name = "centroids/red_pole"
        self.yaw_tracker_client.send_goal_and_block_until_done(goal)

        looking_at_red_pole1 = self.recent_odom

        self.get_logger().warn("move")
        # move forward 0.5 and right by 1
        goal = Move.Goal()
        goal.type = "Relative"
        goal.goal_pose = Pose()
        goal.goal_pose.orientation.w = 1.0
        goal.goal_pose.position.x = 0.5
        goal.goal_pose.position.y = -1.0
        self.move_client.send_goal_and_block_until_done(goal)

        self.get_logger().warn("spot 2")
        self.spot_and_center_red_pole()

        goal = YawTracker.Goal()
        goal.topic_name = "centroids/red_pole"
        self.yaw_tracker_client.send_goal_and_block_until_done(goal)

        looking_at_red_pole2 = self.recent_odom

        red_pole_pose = self.try_find_red_pole(
            looking_at_red_pole1,
            looking_at_red_pole2,
        )

        self.get_logger().warn(f"red pole is at {red_pole_pose}")

        goal = Move.Goal()
        goal.goal_pose.position.x = red_pole_pose[0]
        goal.goal_pose.position.y = red_pole_pose[1] - 0.2
        goal.goal_pose.orientation.w = 1.0
        self.move_client.send_goal_and_block_until_done(goal)

        goal_handle.succeed()
        result = NavChannel.Result()
        result.success = True
        result.message = "NavChannel complete"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavChannelServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
