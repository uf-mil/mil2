import math
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import StartGate


class StartGateServer(Node):
    def __init__(self):
        super().__init__("start_gate")

        # Action server
        self._action_server = ActionServer(
            self,
            StartGate,
            "start_gate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Subscribers

        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        # Publisher for goal poses
        self.goal_pub = self.create_publisher(Pose, "/goal_pose", 10)

        # initialize pose
        self.current_pose = Pose()

    # Called when a goal is received. Parses parameters (from yaml) from mission planner
    def goal_callback(self, goal_request):
        self.total_dist = goal_request.distance
        self.numYaws = goal_request.num_yaws
        self.numRolls = goal_request.num_rolls
        self.numPitch = goal_request.num_pitch

        self.get_logger().info(
            f"Received goal to move through start gate with {self.numYaws} yaws, {self.numRolls} rolls, and {self.numPitch} pitches",
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    # Gets current pose
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_at_goal_pose(self, currentPose, goalPose, acceptableDist=0.05):
        x_dist = currentPose.position.x - goalPose.position.x
        y_dist = currentPose.position.y - goalPose.position.y
        z_dist = currentPose.position.z - goalPose.position.z

        i_dist = currentPose.orientation.x - goalPose.orientation.x
        j_dist = currentPose.orientation.y - goalPose.orientation.y
        k_dist = currentPose.orientation.z - goalPose.orientation.z
        w_dist = currentPose.orientation.w - goalPose.orientation.w

        distance_to_goal = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
        orientation_to_goal = math.sqrt(i_dist**2 + j_dist**2 + k_dist**2 + w_dist**2)
        return distance_to_goal < acceptableDist and orientation_to_goal < 0.1

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            "Executing move through start gate",
        )

        # Align with start gate

        # TODO with vision

        # After aligning with start gate, use params from yaml to determine how far to go and how to r/p/y

        dist_per_rotation = self.total_dist / (
            self.numPitch + self.numRolls + self.numYaws
        )

        # Generate a set of poses to accomplish the desired number of

        goal_poses = []

        # Generate the goal poses for yawing the sub successive 90 degrees
        for yaw in range(self.numPitch):
            pose = Pose()
            pose.position.x = dist_per_rotation * yaw
            pose.position.y = 0
            pose.position.z = 0

            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = math.cos((yaw + 1) * math.pi / 4 - math.pi / 2)
            pose.orientation.w = math.sin((yaw + 1) * math.pi / 4 - math.pi / 2)

            goal_poses.append(pose)

        print(goal_poses)
        for goal_pose in goal_poses:
            self.goal_pub.publish(goal_pose)

            self.get_logger().info(f"Sending goal pose {goal_pose}")
            self.goal_pub.publish(goal_pose)

            near_goal_pose = False
            while not near_goal_pose:
                near_goal_pose = self.check_at_goal_pose(
                    self.current_pose,
                    goal_pose,
                    0.2,
                )

                # slow down the loop and give update odom callback
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.05))
            self.get_logger().info("Arrived at goal pose!")

        self.get_logger().info("Completed start gate!")

        goal_handle.succeed()
        result = StartGateServer.Result()
        result.success = True
        result.message = "Successfully moved through start gate (with style!)"
        time.sleep(1.0)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = StartGateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
