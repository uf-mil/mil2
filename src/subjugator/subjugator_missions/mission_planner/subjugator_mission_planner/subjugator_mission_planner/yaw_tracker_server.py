import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription
from subjugator_msgs.action import YawTracker
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from yolo_msgs.msg import Detection

IMAGE_WIDTH = 840
IMAGE_HEIGHT = 680


class YawTrackerServer(Node):
    def __init__(self):
        super().__init__("yawtracker")

        self.spotted: bool = False

        self.odom_sub_ = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_cb,
            10,
        )
        self.last_odom: Odometry = Odometry()

        self.centroid_sub_: Subscription
        self.last_detection: Detection = Detection()

        self.goal_pose_pub = self.create_publisher(Pose, "goal_pose", 10)

        # Action server
        self._action_server = ActionServer(
            self,
            YawTracker,
            "yawtracker",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def centroid_cb(self, msg: Detection):
        self.last_detection = msg
        self.spotted = True

    def goal_callback(self, goal_request: YawTracker.Goal):
        self.topic_name = goal_request.topic_name
        self.spotted = False
        self.detection_sub = self.create_subscription(
            Detection,
            self.topic_name,
            self.centroid_cb,
            10,
        )

        self.get_logger().info(f"Told to try and center on {self.topic_name}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        self.detection_sub.destroy()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        time.sleep(0.1)
        current_x = self.last_odom.pose.pose.position.x
        current_y = self.last_odom.pose.pose.position.y
        while not self.control_loop(current_x, current_y):
            time.sleep(1)

        # self.detection_sub.destroy()

        goal_handle.succeed()
        result = YawTracker.Result()
        result.success = True
        result.message = "centered on object"
        print("destroyed")
        return result

    def control_loop(self, current_x: float, current_y: float) -> bool:
        if not self.spotted:
            return False

        x_error_pixels = IMAGE_WIDTH / 2 - self.last_detection.bbox.center.position.x
        kp = -2.5 / (2 * IMAGE_WIDTH)

        if abs(x_error_pixels) < 50:
            return True

        print("---------")
        print("x error pixels: ", x_error_pixels)
        yaw_command = -x_error_pixels * kp
        print("yaw command: ", yaw_command)
        print("---------")

        # Get current orientation from odometry
        current_quat = self.last_odom.pose.pose.orientation

        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion(
            [current_quat.x, current_quat.y, current_quat.z, current_quat.w],
        )

        # Apply yaw command
        desired_yaw = yaw + yaw_command

        # Convert back to quaternion
        quat = quaternion_from_euler(roll, pitch, desired_yaw)

        # Create goal pose
        goal_pose = Pose()
        goal_pose.position.x = current_x
        goal_pose.position.y = current_y
        goal_pose.position.z = -0.1  # TODO add current z too :O
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]

        # Publish the goal pose
        self.goal_pose_pub.publish(goal_pose)
        self.spotted = False
        return False


def main(args=None):
    rclpy.init(args=args)
    node = YawTrackerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
