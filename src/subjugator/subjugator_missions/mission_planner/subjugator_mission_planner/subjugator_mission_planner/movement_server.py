import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from subjugator_msgs.action import Movement


class MovementServer(Node):
    def __init__(self):
        super().__init__("movement_server")

        # Action server
        self._action_server = ActionServer(
            self,
            Movement,
            "movement_server",
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

        distance_to_goal = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
        self.get_logger().info(f"dist: {distance_to_goal}")
        return distance_to_goal < acceptableDist

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            f"Executing move to {self.movementGoal}, {self.movementType}",
        )

        # Generate goal poses for orbit
        goal_pose = self.movementGoal

        self.goal_pub.publish(goal_pose)

        near_goal_pose = False
        while not near_goal_pose:
            near_goal_pose = self.check_at_goal_pose(self.current_pose, goal_pose, 0.2)

            # slow down the loop
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.05))

        self.get_logger().info("Arrived at goal pose!")

        self.get_logger().info("Completed movement!")

        goal_handle.succeed()
        result = Movement.Result()
        result.success = True
        result.message = "Successfully moved to goal pose"
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
