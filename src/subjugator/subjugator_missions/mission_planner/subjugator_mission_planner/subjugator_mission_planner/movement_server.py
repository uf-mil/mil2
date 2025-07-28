import math
import time

import mission_support
import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from subjugator_msgs.action import Move


class MovementServer(Node):
    def __init__(self):
        super().__init__("move")

        # Action server
        self._action_server = ActionServer(
            self,
            Move,
            "move",
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

    # Called when a goal is received. Determines if using vision or dead-reckoning to orbit target
    def goal_callback(self, goal_request):
        self.movementType = goal_request.type
        self.movementGoal = goal_request.goal_pose
        self.get_logger().info(
            f"Received goal to move to {self.movementGoal}, {self.movementType}",
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
            f"Executing move to {self.movementGoal}, {self.movementType}",
        )

        # Generate goal poses for orbit
        if self.movementType == "Relative":
            # Convert current orientation to scipy Rotation object
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ]
            current_rot = R.from_quat(current_quat)

            # Relative position from goal
            rel_position = np.array(
                [
                    self.movementGoal.position.x,
                    self.movementGoal.position.y,
                    self.movementGoal.position.z,
                ],
            )

            # Rotate relative position into world frame
            rotated_position = current_rot.apply(rel_position)

            # Add to current position
            goal_position = (
                np.array(
                    [
                        self.current_pose.position.x,
                        self.current_pose.position.y,
                        self.current_pose.position.z,
                    ],
                )
                + rotated_position
            )

            # Relative orientation (as Rotation)
            rel_quat = [
                self.movementGoal.orientation.x,
                self.movementGoal.orientation.y,
                self.movementGoal.orientation.z,
                self.movementGoal.orientation.w,
            ]
            rel_rot = R.from_quat(rel_quat)

            # Compose rotations: world_rot * relative_rot
            goal_rot = current_rot * rel_rot
            goal_quat = goal_rot.as_quat()  # [x, y, z, w]

            # Create absolute Pose
            goal_pose = Pose()
            goal_pose.position.x = goal_position[0]
            goal_pose.position.y = goal_position[1]
            goal_pose.position.z = goal_position[2]
            goal_pose.orientation.x = goal_quat[0]
            goal_pose.orientation.y = goal_quat[1]
            goal_pose.orientation.z = goal_quat[2]
            goal_pose.orientation.w = goal_quat[3]

            self.get_logger().info(f"Converted move to absolute: {goal_pose}")
        else:
            goal_pose = self.movementGoal

        self.goal_pub.publish(goal_pose)

        near_goal_pose = False
        while not near_goal_pose:
            near_goal_pose = mission_support.check_at_goal_pose(
                self.current_pose,
                goal_pose,
                0.2,
            )

            # slow down the loop
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.05))

        self.get_logger().info("Arrived at goal pose!")

        self.get_logger().info("Completed movement!")

        goal_handle.succeed()
        result = Move.Result()
        result.success = True
        result.message = "Successfully moved to goal pose"
        time.sleep(1.0)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MovementServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
