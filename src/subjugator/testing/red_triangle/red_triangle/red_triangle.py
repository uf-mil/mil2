# todo you need to center on the thing too :(

import math
from copy import deepcopy

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from subjugator_msgs.msg import Centroid
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class RedTriangle(Node):
    def __init__(self):
        super().__init__("red_triangle")

        # centroid cb
        self._centroid_cb = self.create_subscription(
            Centroid,
            "centroids/red",
            self.centroid_cb,
            10,
        )
        self.recent_centroid = Centroid()
        self.centroid_cb_counter = 0  # for detecting if we are seeing red rn

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

    def centroid_cb(self, msg: Centroid):
        self.recent_centroid = msg
        self.centroid_cb_counter += 1
        # what is a Centroid type? Here:
        # self.recent_centroid.centroid_x
        # self.recent_centroid.centroid_y
        # self.recent_centroid.image_width
        # self.recent_centroid.image_height

    def odom_cb(self, msg: Odometry):
        self.recent_odom = msg

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

    def take_current_odom_and_yaw_by_n_degrees(self, n) -> Pose:
        # Get current pose from odometry
        current_pose = self.recent_odom.pose.pose
        current_q = current_pose.orientation

        # Convert current orientation to euler angles
        current_euler = euler_from_quaternion(
            [current_q.x, current_q.y, current_q.z, current_q.w],
        )

        # Add 10 degrees to yaw (left rotation is positive in ROS standard)
        rotation_increment = math.radians(n)  # 10 degrees in radians
        new_yaw = current_euler[2] + rotation_increment

        # Convert back to quaternion
        new_quat = quaternion_from_euler(current_euler[0], current_euler[1], new_yaw)

        # Create new pose with same position but rotated orientation
        new_pose = Pose()
        new_pose.position = current_pose.position
        new_pose.orientation = Quaternion(
            x=new_quat[0],
            y=new_quat[1],
            z=new_quat[2],
            w=new_quat[3],
        )  # get our current odom and rotate left 10 degrees

        return new_pose

    def rotate_left_until_15_centroids(self):
        self.centroid_cb_counter = (
            0  # reset counter, it will increase when red is in frame
        )
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # rotate left 10 degrees
            new_pose = self.take_current_odom_and_yaw_by_n_degrees(10)
            self.send_goal_and_wait(new_pose)
            if (
                self.centroid_cb_counter > 15
            ):  # implies we have 15 frames of red implies we are seeing pvc pipe
                return

    # this function assumes the thing is already in frame
    def attempt_to_triangulate(self) -> Point:
        # first, store current position and centroid
        pose1: Odometry = deepcopy(self.recent_odom)
        centroid1: Centroid = deepcopy(self.recent_centroid)

        # next, move right in odom
        new_pose = Pose()
        new_pose.position = self.recent_odom.pose.pose.position
        new_pose.orientation = self.recent_odom.pose.pose.orientation
        new_pose.position.y -= 1.5  # move right by 1.5 meters

        # block until goal is good
        self.send_goal_and_wait(new_pose)

        # rotate left until we see the object again
        self.rotate_left_until_15_centroids()

        # second pose
        pose2: Odometry = deepcopy(self.recent_odom)
        centroid2: Centroid = deepcopy(self.recent_centroid)

        return self.generate_triangulation_guess(pose1, pose2, centroid1, centroid2)

    # sorry, chat wrote this entire function
    def generate_triangulation_guess(
        self,
        pose1: Odometry,
        pose2: Odometry,
        c1: Centroid,
        c2: Centroid,
    ) -> Point:
        """
        Triangulate the 3D position of an object using two different viewpoints.

        Args:
            pose1: First robot pose when observing the object
            pose2: Second robot pose when observing the object
            c1: Centroid observation from first position
            c2: Centroid observation from second position

        Returns:
            Point: Estimated 3D position of the object
        """
        # Extract camera parameters (these should be adjusted based on your camera)
        # Field of view in radians
        fov_h = math.radians(60)  # Horizontal field of view
        fov_v = math.radians(45)  # Vertical field of view

        # Function to convert centroid to direction vector in robot frame
        def centroid_to_direction(centroid, pose):
            # Normalize centroid coordinates to [-1, 1]
            norm_x = (centroid.centroid_x / centroid.image_width) * 2 - 1
            norm_y = (centroid.centroid_y / centroid.image_height) * 2 - 1

            # Calculate angles based on normalized coordinates and FOV
            angle_h = norm_x * (fov_h / 2)
            angle_v = norm_y * (fov_v / 2)

            # Create direction vector in camera frame
            # Assuming camera points forward along x-axis
            # z is up, y is left (standard ROS convention)
            camera_dir = np.array(
                [
                    math.cos(angle_v) * math.cos(angle_h),  # x - forward
                    math.cos(angle_v) * math.sin(angle_h),  # y - left
                    math.sin(angle_v),  # z - up
                ],
            )

            # Convert quaternion to rotation matrix to transform from camera to world frame
            q = pose.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            rotation_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]

            # Transform direction vector from camera frame to world frame
            world_dir = np.dot(rotation_matrix, camera_dir)

            # Normalize the direction vector
            return world_dir / np.linalg.norm(world_dir)

        # Get positions and direction vectors
        pos1 = np.array(
            [
                pose1.pose.pose.position.x,
                pose1.pose.pose.position.y,
                pose1.pose.pose.position.z,
            ],
        )
        pos2 = np.array(
            [
                pose2.pose.pose.position.x,
                pose2.pose.pose.position.y,
                pose2.pose.pose.position.z,
            ],
        )

        dir1 = centroid_to_direction(c1, pose1)
        dir2 = centroid_to_direction(c2, pose2)

        # Line 1: pos1 + t1 * dir1
        # Line 2: pos2 + t2 * dir2
        # Find closest points between these two lines

        # Vector connecting the two camera positions
        v = pos2 - pos1

        # Calculate dot products
        d1_dot_d2 = np.dot(dir1, dir2)
        d1_dot_v = np.dot(dir1, v)
        d2_dot_v = np.dot(dir2, v)

        # Calculate denominators
        denom = 1 - d1_dot_d2**2

        # Handle parallel lines
        if abs(denom) < 1e-6:
            self.get_logger().warning("Triangulation failed: rays are nearly parallel")
            # Return midpoint between cameras as fallback
            midpoint = (pos1 + pos2) / 2
            result = Point()
            result.x, result.y, result.z = midpoint
            return result

        # Calculate parameters for closest points on each line
        t1 = (d1_dot_v - d2_dot_v * d1_dot_d2) / denom
        t2 = (d1_dot_v * d1_dot_d2 - d2_dot_v) / denom

        # Calculate closest points on each ray
        point1 = pos1 + dir1 * t1
        point2 = pos2 + dir2 * t2

        # Calculate squared distance between the closest points
        distance_squared = np.sum((point1 - point2) ** 2)

        # Check if the rays are too far apart
        if distance_squared > 1.0:  # 1.0 meter squared threshold
            self.get_logger().warning(
                f"Triangulation may be inaccurate: rays don't intersect closely (dist²={distance_squared})",
            )

        # Calculate midpoint as the triangulated position
        triangulated = (point1 + point2) / 2

        # Create and return Point message
        result = Point()
        result.x = float(triangulated[0])
        result.y = float(triangulated[1])
        result.z = float(triangulated[2])

        self.get_logger().info(
            f"Triangulated position: ({result.x}, {result.y}, {result.z})",
        )
        return result


def main():
    rclpy.init()
    rt = RedTriangle()
    rclpy.spin(rt)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
