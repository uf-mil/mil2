import asyncio
import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String

from subjugator_mission_planner.action import NavigateAroundObject


class NavigateAroundObjectServer(Node):
    def __init__(self):
        super().__init__("navigate_around_object_server")

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateAroundObject,
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

        # Internal state
        self.current_detection = None
        self.current_robot_pose = None

        # Initialize relative position and heading of target object (need to get from cams)
        self.distance_to_target = 0.0
        self.heading_to_target = 0.0

    def goal_callback(self, goal_request):
        # TODO ADD CAPABILITY TO REJECT NAVIGATE AROUND ACTION IF OBJECT NOT DETECTED
        self.get_logger().info(
            f"Received goal to navigate around {goal_request.object}",
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def perception_callback(self, msg):
        self.current_detection = msg.data
        # TODO NEED TO UPDATE DISTANCE AND HEADING FROM PERCEPTION
        self.distance_to_target = 0.0
        self.heading_to_target = 0

    def odom_callback(self, msg):
        self.current_robot_pose = msg.pose.pose

    def generate_around_poses(self, robot_pose):
        poses = []

        # Approximate object position in map frame
        object_x = robot_pose.position.x + self.relative_distance * math.cos(
            self.relative_bearing,
        )
        object_y = robot_pose.position.y + self.relative_distance * math.sin(
            self.relative_bearing,
        )
        object_z = robot_pose.position.z

        # distance to orbit object at
        radius = 1.0

        for angle_deg in [0, 90, 180, 270]:
            angle_rad = math.radians(angle_deg)
            pose = Pose()
            pose.position.x = object_x + radius * math.cos(angle_rad)
            pose.position.y = object_y + radius * math.sin(angle_rad)
            pose.position.z = object_z
            pose.orientation.w = 1.0
            poses.append(pose)

        return poses

    async def execute_callback(self, goal_handle):
        target_object = goal_handle.request.object
        self.get_logger().info(f"Executing NavigateAroundObject for: {target_object}")

        object_detected = False

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateAroundObject.Result()
                result.success = False
                result.message = "Canceled"
                return result

            if self.current_detection == target_object:
                object_detected = True
                break

            self.get_logger().info("Waiting for object detection...")
            await asyncio.sleep(0.5)

        if not object_detected:
            goal_handle.abort()
            result = NavigateAroundObject.Result()
            result.success = False
            result.message = "Object not detected"
            return result

        # Generate and publish poses
        if self.current_robot_pose is None:
            self.get_logger().error("No robot pose available!")
            goal_handle.abort()
            result = NavigateAroundObject.Result()
            result.success = False
            result.message = "Missing robot pose"
            return result

        waypoints = self.generate_around_poses(self.current_robot_pose)

        for pose in waypoints:
            self.goal_pub.publish(pose)
            self.get_logger().info(
                f"Published pose: x={pose.position.x:.2f}, y={pose.position.y:.2f}",
            )
            await asyncio.sleep(3.0)  # give time to move to pose

        goal_handle.succeed()
        result = NavigateAroundObject.Result()
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
