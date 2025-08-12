# todo you need to center on the thing too :(

from rclpy.node import Node
import rclpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from subjugator_msgs.msg import Centroid
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from copy import deepcopy

import numpy as np
import math
import tf_transformations

class NavChannel(Node):
    def __init__(self):
        super().__init__('nav_channel')

        self.spotted = False

        # centroid cb
        self._centroid_cb = self.create_subscription(Centroid, "centroids/red", self.centroid_cb, 10)
        self.recent_centroid = Centroid()
        self.centroid_cb_counter = 0 # for detecting if we are seeing red rn

        # odom cb
        self._odom_cb = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, 10)
        self.recent_odom = Odometry()

        # goal pub
        self. goal_pub = self.create_publisher(Pose, "goal_pose", 10)

    def centroid_cb(self, msg: Centroid):
        self.recent_centroid = msg
        self.centroid_cb_counter += 1
        # what is a Centriod type? Here:
        # self.recent_centroid.centroid_x
        # self.recent_centroid.centroid_y
        # self.recent_centroid.image_width
        # self.recent_centroid.image_height

    def odom_cb(self, msg: Odometry):
        self.recent_odom = msg

    def control_loop(self) -> bool:
        rclpy.spin_once(self, timeout_sec=1)
        if not self.spotted:
            return False

        x_error_pixels = (
            self.recent_centroid.image_width / 2 - self.recent_centroid.centroid_x
        )

        if x_error_pixels < 50:
            return True

        kp = 2.5 / (2 * self.recent_centroid.image_width)

        print("---------")
        print(x_error_pixels)
        yaw_command = -x_error_pixels * kp
        print(yaw_command)
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
        return False

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
            position_error = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Calculate orientation error using euler angles
            current_q = self.recent_odom.pose.pose.orientation
            goal_q = goal.orientation
            # Convert quaternions to euler angles (roll, pitch, yaw)
            current_euler = euler_from_quaternion([current_q.x, current_q.y, current_q.z, current_q.w])
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
            if position_error < position_tolerance and orientation_error < orientation_tolerance:
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

    def take_current_odom_and_move_in_plus_y(self, n) -> Pose:
        # Get current pose from odometry
        current_pose = self.recent_odom.pose.pose
        current_q = current_pose.orientation
        
        # Create new pose with same position but rotated orientation
        new_pose = Pose()
        new_pose.position = deepcopy(current_pose.position)
        new_pose.position.y += n
        new_pose.orientation = current_q

        return new_pose

    def rotate_left_until_15_centroids(self):
        self.centroid_cb_counter = 0 # reset counter, it will increase when red is in frame
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # rotate left 10 degrees
            new_pose = self.take_current_odom_and_yaw_by_n_degrees(10)
            self.send_goal_and_wait(new_pose)
            if self.centroid_cb_counter > 15: # implies we have 15 frames of red implies we are seeing pvc pipe
                return

    def slow_forward_until_centroid(self):
        self.centroid_cb_counter = 0 # reset counter, it will increase when red is in frame
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # rotate left 10 degrees
            new_pose = self.take_current_odom_and_move_in_plus_y(0.45)
            self.send_goal_and_wait(new_pose)
            if self.centroid_cb_counter > 15: # implies we have 15 frames of red implies we are seeing pvc pipe
                return
    def do_nav_channel(self):
        # turn until red in frame
        self.rotate_left_until_15_centroids()

        # center on red
        while True:
            if self.control_loop() == True:
                break

        # store current bearing
        looking_at_red_pole: Odometry = deepcopy(self.recent_odom)

        # turn left a little (but also yaw right by 90
        new_pose = self.take_current_odom_and_yaw_by_n_degrees(13-90)
        self.send_goal_and_wait(new_pose)

        # slowly go straight until red in view
        self.slow_forward_until_centroid()

def main():
    rclpy.init()
    nc = NavChannel()
    rclpy.spin(nc)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
