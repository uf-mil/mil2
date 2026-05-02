import copy
import math

import rclpy
import rclpy.duration
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Move
from tf2_ros import Buffer, TransformListener

TIMEOUT_LENGTH = 0.5
VELOCITY_TOLERANCE = 1e-3


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
        self.goal_pub = self.create_publisher(Pose, "goal/trajectory_relative", 1)

        # initialize pose
        self.current_pose = Pose()
        self.last_pose = None

        # Timeout initializer
        self.last_recorded_pose = None
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.start_timeout = False
        self.terminate = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def check_at_goal_pose(self, currentPose, goalPose, acceptableDist=0.5):
        x_dist = currentPose.position.x - goalPose.position.x
        y_dist = currentPose.position.y - goalPose.position.y
        z_dist = currentPose.position.z - goalPose.position.z

        i_dist = currentPose.orientation.x - goalPose.orientation.x
        j_dist = currentPose.orientation.y - goalPose.orientation.y
        k_dist = currentPose.orientation.z - goalPose.orientation.z
        w_dist = currentPose.orientation.w - goalPose.orientation.w

        distance_to_goal = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
        orientation_to_goal = math.sqrt(i_dist**2 + j_dist**2 + k_dist**2 + w_dist**2)

        if self.last_recorded_pose:

            x_step_dist = currentPose.position.x - self.last_recorded_pose.position.x
            y_step_dist = currentPose.position.y - self.last_recorded_pose.position.y
            z_step_dist = currentPose.position.z - self.last_recorded_pose.position.z

            i_step_dist = (
                currentPose.orientation.x - self.last_recorded_pose.orientation.x
            )
            j_step_dist = (
                currentPose.orientation.y - self.last_recorded_pose.orientation.y
            )
            k_step_dist = (
                currentPose.orientation.z - self.last_recorded_pose.orientation.z
            )
            w_step_dist = (
                currentPose.orientation.w - self.last_recorded_pose.orientation.w
            )

            self.velocity = math.sqrt(x_step_dist**2 + y_step_dist**2 + z_step_dist**2)
            self.angular_velocity = math.sqrt(
                i_step_dist**2 + j_step_dist**2 + k_step_dist**2 + w_step_dist**2,
            )

            has_not_moved = (
                self.velocity < VELOCITY_TOLERANCE
                and self.angular_velocity < VELOCITY_TOLERANCE
            )

            if self.start_timeout and has_not_moved:
                self.get_logger().info(
                    f"Terminating at {distance_to_goal} distance and {orientation_to_goal} orientation because {self.velocity:.8f} and {self.angular_velocity:.8f} are less than {VELOCITY_TOLERANCE}",
                )
                self.terminate = True

            self.start_timeout = has_not_moved

        self.last_recorded_pose = copy.deepcopy(currentPose)

        return distance_to_goal < acceptableDist and orientation_to_goal < 0.05

    def execute_callback(self, goal_handle):
        result = Move.Result()

        self.get_logger().info(
            f"Executing move to {self.movementGoal}, {self.movementType}",
        )

        # Generate goal poses
        if self.movementType == "Reset":
            self.get_logger().info("Resetting goal pose reference")

            goal_pose = Pose()
            goal_pose.position.x = 0.0
            goal_pose.position.y = 0.0
            goal_pose.position.z = 0.0
            goal_pose.orientation.x = 0.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = 1.0

            self.last_pose = goal_pose

            goal_handle.succeed()
            result.success = True
            result.message = "Reset reference"
            return result
        else:
            goal_pose = copy.deepcopy(self.current_pose)
            goal_pose.position.x += self.movementGoal.position.x
            goal_pose.position.y += self.movementGoal.position.y
            goal_pose.position.z += self.movementGoal.position.z
            goal_pose.orientation.x += self.movementGoal.orientation.x
            goal_pose.orientation.y += self.movementGoal.orientation.y
            goal_pose.orientation.z += self.movementGoal.orientation.z
            goal_pose.orientation.w += self.movementGoal.orientation.w

        self.goal_pub.publish(self.movementGoal)

        result = Move.Result()

        near_goal_pose = False
        while not near_goal_pose:
            near_goal_pose = self.check_at_goal_pose(self.current_pose, goal_pose, 0.2)

            if self.terminate:
                self.get_logger().info("Oh no! I got stuck!")
                self.last_pose = self.current_pose
                goal_handle.abort()
                result.success = False
                result.message = "Got stuck..."

                # Reset flags
                self.terminate = False
                self.start_timeout = False

                return result

            # slow down the loop
            self.get_clock().sleep_for(
                rclpy.duration.Duration(
                    seconds=TIMEOUT_LENGTH if self.start_timeout else 0.05,
                ),
            )

        self.get_logger().info("Arrived at goal pose!")
        self.get_logger().info("Completed movement!")

        goal_handle.succeed()
        self.last_pose = goal_pose
        result.success = True
        result.message = "Successfully moved to goal pose"

        # Reset flags
        self.terminate = False
        self.start_timeout = False

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
