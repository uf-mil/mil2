import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription
from scipy.spatial.transform import Rotation as R
from subjugator_msgs.action import ZYTracker
from yolo_msgs.msg import Detection

IMAGE_WIDTH = 840
IMAGE_HEIGHT = 680


class ZyTrackerServer(Node):
    def __init__(self):
        super().__init__("zytracker")

        self.spotted: bool = False

        self.detection_sub: Subscription
        self.last_detection: Detection = Detection()

        self.detection_sub = self.create_subscription(
            Detection,
            "centroids/swordfish",
            self.centroid_cb,
            10,
        )

        self.goal_pose_pub = self.create_publisher(Pose, "goal_pose", 10)

        # sub to odom
        self.odom_sub_ = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_cb,
            10,
        )
        self.last_odom = Odometry()

        # Action server
        self._action_server = ActionServer(
            self,
            ZYTracker,
            "zytracker",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def sleep_for(self, time: float):
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=time)

        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks while waiting

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def centroid_cb(self, msg: Detection):
        self.last_detection = msg
        self.spotted = True

    def goal_callback(self, goal_request: ZYTracker.Goal):
        self.topic_name = goal_request.topic_name
        self.spotted = False
        self.get_logger().warn(f"starting detections on {self.topic_name}")

        self.get_logger().info(f"Told to try and yz servo on {self.topic_name}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        # self.detection_sub.destroy()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        time.sleep(0.1)
        while not self.control_loop():
            time.sleep(1)
        self.get_logger().warn("centered on thing :)")

        # self.detection_sub.destroy()

        goal_handle.succeed()
        result = ZYTracker.Result()
        result.success = True
        result.message = "zy centered on object"
        print("destroyed")
        return result

    def relative_move(self, x=0.0, y=0.0, z=0.0, i=0.0, j=0.0, k=0.0, w=1.0):
        # Convert current orientation to scipy Rotation object
        current_quat = [
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
            self.last_odom.pose.pose.orientation.w,
        ]
        current_rot = R.from_quat(current_quat)

        # Relative position from goal
        rel_position = np.array(
            [x, y, z],
        )

        # Rotate relative position into world frame
        rotated_position = current_rot.apply(rel_position)

        # Add to current position
        goal_position = (
            np.array(
                [
                    self.last_odom.pose.pose.position.x,
                    self.last_odom.pose.pose.position.y,
                    self.last_odom.pose.pose.position.z,
                ],
            )
            + rotated_position
        )

        # Relative orientation (as Rotation)
        rel_quat = [i, j, k, w]
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

        self.goal_pose_pub.publish(goal_pose)

    def control_loop(self) -> bool:
        if not self.spotted:
            self.get_logger().warn("doing nothing since no detection")
            return False

        x_error_pixels = IMAGE_WIDTH / 2 - self.last_detection.bbox.center.position.x
        y_error_pixels = IMAGE_HEIGHT / 2 - self.last_detection.bbox.center.position.y
        kp_x = -2.5 / (2 * IMAGE_WIDTH)
        kp_y = -2.5 / (2 * IMAGE_HEIGHT)

        if abs(x_error_pixels) < 35 and abs(y_error_pixels) < 35:
            self.get_logger().warn("found and centered!!!!")
            return True

        print("---------")
        y_command = -x_error_pixels * kp_x
        z_command = -y_error_pixels * kp_y
        print("y command: ", y_command)
        print("z command: ", z_command)
        print("---------")

        # perform a relative move
        self.relative_move(0.0, y_command, z_command)

        self.spotted = False
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ZyTrackerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
