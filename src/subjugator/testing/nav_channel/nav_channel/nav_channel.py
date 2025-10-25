import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from yolo_msgs.msg import Detection

IMAGE_WIDTH = 840
IMAGE_HEIGHT = 680


class NavChannel(Node):
    def __init__(self):
        super().__init__("nav_channel")

        # centroid cb
        self._detection_sub = self.create_subscription(
            Detection,
            "centroids/Green",
            self.detection_cb,
            10,
        )
        self.recent_detection: Detection = Detection()
        self.detection_cb_counter = 0  # for detecting if we are seeing something rn
        self.spotted = False

        # odom cb
        self._odom_cb = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_cb,
            10,
        )
        self.recent_odom = Odometry()

        # goal pub
        self.goal_pub = self.create_publisher(Pose, "goal_pose", 10)

    def detection_cb(self, msg: Detection):
        self.recent_detection = msg
        self.detection_cb_counter += 1
        self.spotted = True

    def odom_cb(self, msg: Odometry):
        self.recent_odom = msg

    # generate a move in base_link, adam made this :)
    def move_relative(self, movement_goal: Pose) -> Pose:
        goal_pose = Pose()

        current_quat = [
            self.recent_odom.pose.pose.orientation.x,
            self.recent_odom.pose.pose.orientation.y,
            self.recent_odom.pose.pose.orientation.z,
            self.recent_odom.pose.pose.orientation.w,
        ]
        current_rot = R.from_quat(current_quat)

        # Relative position from goal
        rel_position = np.array(
            [
                movement_goal.position.x,
                movement_goal.position.y,
                movement_goal.position.z,
            ],
        )

        # Rotate relative position into world frame
        rotated_position = current_rot.apply(rel_position)

        # Add to current position
        goal_position = (
            np.array(
                [
                    self.recent_odom.pose.pose.position.x,
                    self.recent_odom.pose.pose.position.y,
                    self.recent_odom.pose.pose.position.z,
                ],
            )
            + rotated_position
        )

        # Relative orientation (as Rotation)
        rel_quat = [
            movement_goal.orientation.x,
            movement_goal.orientation.y,
            movement_goal.orientation.z,
            movement_goal.orientation.w,
        ]
        rel_rot = R.from_quat(rel_quat)

        # Compose rotations: world_rot * relative_rot
        goal_rot = current_rot * rel_rot
        goal_quat = goal_rot.as_quat(True)  # [x, y, z, w]

        # Create absolute Pose
        goal_pose = Pose()
        goal_pose.position.x = goal_position[0]
        goal_pose.position.y = goal_position[1]
        goal_pose.position.z = goal_position[2]
        goal_pose.orientation.x = goal_quat[0]
        goal_pose.orientation.y = goal_quat[1]
        goal_pose.orientation.z = goal_quat[2]
        goal_pose.orientation.w = goal_quat[3]

        return goal_pose

    def control_loop(self):
        if not self.spotted:
            return

        x_error_pixels = IMAGE_WIDTH / 2 - self.recent_detection.bbox.center.position.x
        kp = -2.5 / (2 * IMAGE_WIDTH)

        print("---------")
        print("x error pixels: ", x_error_pixels)
        yaw_command = -x_error_pixels * kp
        print("yaw command: ", yaw_command)
        print("---------")

        # Get current orientation from odometry
        current_quat = self.recent_odom.pose.pose.orientation

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
        goal_pose.position.x = 0.0
        goal_pose.position.y = 0.0
        goal_pose.position.z = -0.1
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]

        # Publish the goal pose
        self.goal_pub.publish(goal_pose)
        self.spotted = False

    def send_goal_and_wait(self, goal: Pose):
        self.goal_pub.publish(goal)
        position_tolerance = 0.2  # in meters
        orientation_tolerance = 0.1  # in radians

        while True:
            rclpy.spin_once(self, timeout_sec=0.25)
            # Calculate position error
            current_pos = self.recent_odom.pose.pose.position
            dx = current_pos.x - goal.position.x
            dy = current_pos.y - goal.position.y
            dz = current_pos.z - goal.position.z
            position_error = math.sqrt(dx * dx + dy * dy + dz * dz)

            # Calculate orientation error using euler angles
            current_q = self.recent_odom.pose.pose.orientation
            goal_q = goal.orientation
            # Convert quaternions to euler angles (roll, pitch, yaw)
            current_euler = euler_from_quaternion(
                [current_q.x, current_q.y, current_q.z, current_q.w],
            )
            goal_euler = euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])

            roll_error = abs(current_euler[0] - goal_euler[0])
            pitch_error = abs(current_euler[1] - goal_euler[1])
            yaw_error = abs(current_euler[2] - goal_euler[2])

            # Handle angle wrapping for all angles
            roll_error = min(roll_error, 2 * math.pi - roll_error)
            pitch_error = min(pitch_error, 2 * math.pi - pitch_error)
            yaw_error = min(yaw_error, 2 * math.pi - yaw_error)

            # choose max
            orientation_error = max(roll_error, pitch_error, yaw_error)

            # Check if close enough to goal
            if (
                position_error < position_tolerance
                and orientation_error < orientation_tolerance
            ):
                self.get_logger().info("Goal reached!")
                break

    # -180 < degrees < 180 or ur gonna PMO
    def take_current_odom_and_yaw_by_n_degrees(self, degrees: int) -> Pose:
        yaw_request = Pose()
        yaw_request.orientation.z = math.sin(degrees / 2)
        yaw_request.orientation.w = math.cos(degrees / 2)

        return self.move_relative(yaw_request)

    def take_current_odom_and_move_in_plus_y(self, base_link_y_offset) -> Pose:
        y_translate_request = Pose()
        y_translate_request.orientation.w = 1
        y_translate_request.position.y = base_link_y_offset

        return self.move_relative(y_translate_request)

    def rotate_left_until_15_centroids(self):
        self.detection_cb_counter = (
            0  # reset counter, it will increase when red is in frame
        )
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # rotate left 10 degrees
            new_pose = self.take_current_odom_and_yaw_by_n_degrees(10)
            self.send_goal_and_wait(new_pose)
            if (
                self.detection_cb_counter > 15
            ):  # implies we have 15 frames of red implies we are seeing pvc pipe
                return

    def slow_forward_until_centroid(self):
        self.detection_cb_counter = (
            0  # reset counter, it will increase when red is in frame
        )
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # rotate left 10 degrees
            new_pose = self.take_current_odom_and_move_in_plus_y(0.45)
            self.send_goal_and_wait(new_pose)
            if (
                self.detection_cb_counter > 15
            ):  # implies we have 15 frames of red implies we are seeing pvc pipe
                return

    def do_nav_channel(self):
        # turn until red in frame
        self.rotate_left_until_15_centroids()

        # center on red
        while True:
            if self.control_loop():
                break

        # store current bearing (bearing where we are looking at the red pole
        # looking_at_red_pole: Odometry = deepcopy(self.recent_odom)

        # turn left a little (but also yaw right by 90
        new_pose = self.take_current_odom_and_yaw_by_n_degrees(13 - 90)
        self.send_goal_and_wait(new_pose)

        # slowly go straight until red in view
        self.slow_forward_until_centroid()


def main():
    rclpy.init()
    nc = NavChannel()
    rclpy.spin_once(nc, timeout_sec=0.25)
    nc.do_nav_channel()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
