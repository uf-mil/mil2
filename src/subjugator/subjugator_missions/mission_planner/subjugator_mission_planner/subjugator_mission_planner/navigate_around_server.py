import math

import rclpy
import rclpy.task
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String
from subjugator_msgs.action import NavigateAround


class NavigateAroundObjectServer(Node):
    def __init__(self):
        super().__init__("navigate_around_object_server")

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateAround,
            "navigate_around_object",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Subscribers
        self.create_subscription(
            String,
            "/detected_objects",
            self.perception_callback,
            10,
        )
        self.create_subscription(Odometry, "/odometry_filtered", self.odom_callback, 10)

        # Publisher for goal poses
        self.goal_pub = self.create_publisher(Pose, "/goal_pose", 10)

        # initialize pose
        self.current_pose = Pose()

    # Called when a goal is received. Determines if using vision or dead-reckoning to orbit target
    def goal_callback(self, goal_request):
        self.distance_to_orbit = goal_request.distance
        if goal_request.object == "None":
            self.get_logger().info(
                f"Received no object to orbit. Goal set to orbit about point ahead of current pose by {goal_request.distance}",
            )
            self.use_vision = False
        else:
            self.get_logger().info(
                f"Received goal to navigate around {goal_request.object} at a distance of {goal_request.distance}",
            )
            self.use_vision = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def perception_callback(self, msg):
        # TODO depending on how we implement perception
        pass

    # Gets current pose
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def generate_poses(self, currentPose):
        # Generate array of 4 poses that will complete an orbit around the target:
        #                2
        #                |
        #                |
        #        3 - - Target - - 1
        #                |
        #                |
        #               0/4
        poses = []

        # Get object position relative to current pose
        target_x = currentPose.position.x
        target_y = currentPose.position.y + self.distance_to_orbit
        target_z = currentPose.position.z

        # Find poses for completing orbit
        for angle_deg in [90, 180, 270, 0]:
            angle_rad = math.radians(angle_deg)
            pose = Pose()
            pose.position.x = target_x + self.distance_to_orbit * math.sin(angle_rad)
            pose.position.y = target_y + self.distance_to_orbit * math.cos(angle_rad)
            pose.position.z = target_z
            pose.orientation.w = 1.0
            poses.append(pose)

        return poses

    def check_at_goal_pose(self, currentPose, goalPose, acceptableDist=0.1):
        x_dist = currentPose.position.x - goalPose.position.x
        y_dist = currentPose.position.y - goalPose.position.y
        z_dist = currentPose.position.z - goalPose.position.z

        distance_to_goal = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
        return distance_to_goal < acceptableDist

    def execute_callback(self, goal_handle):

        orbit_distance = goal_handle.request.distance
        # If using vision to navigate around object, keep object at center of camera frame
        if self.use_vision:
            target_object = goal_handle.request.object

            self.get_logger().info(
                f"Executing Navigate Around for: {target_object} at distance {orbit_distance} using vision",
            )

        # If using dead-reckoning to navigate around object, generate a set of goal poses, then follow them around the object.
        else:
            self.get_logger().info(
                f"Executing Navigate Around at distance {orbit_distance} using dead-reckoning",
            )

            # Generate goal poses for orbit
            goal_poses = self.generate_poses(self.current_pose)

            # Move to the poses
            for pose in goal_poses:
                self.goal_pub.publish(pose)
                self.get_logger().info(
                    f"Published pose: x={pose.position.x:.2f}, y={pose.position.y:.2f},z={pose.position.z:.2f}",
                )
                near_goal_pose = False
                while not near_goal_pose:
                    near_goal_pose = self.check_at_goal_pose(self.current_pose, pose)

                    # slow down the loop
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.05))
                self.get_logger().info("Arrived at goal pose!")
            self.get_logger().info("Completed orbit!")

        goal_handle.succeed()
        result = NavigateAround.Result()
        result.success = True
        result.message = "Successfully navigated around object"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateAroundObjectServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
