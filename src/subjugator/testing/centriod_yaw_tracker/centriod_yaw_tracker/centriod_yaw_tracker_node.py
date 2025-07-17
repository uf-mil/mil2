import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from subjugator_msgs.msg import Centriod
from geometry_msgs.msg import Pose

from tf_transformations import euler_from_quaternion, quaternion_from_euler

class CentriodYawTracker(Node):
    def __init__(self):
        super().__init__("centriod_yaw_tracker")

        self.odom_sub_ = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, 10)
        self.last_odom: Odometry = Odometry()

        self.centriod_sub_ = self.create_subscription(Centriod, "centriods/green", self.centriod_cb, 10)
        self.last_centriod: Centriod = Centriod()

        self.goal_pose_pub = self.create_publisher(Pose, "goal_pose", 10)

        self.timer = self.create_timer(1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg
    def centriod_cb(self, msg: Centriod):
        self.last_centriod = msg

    def control_loop(self):
        x_error_pixels = self.last_centriod.image_width - self.last_centriod.centroid_x
        kp = 1/self.last_centriod.image_width

        yaw_command = -x_error_pixels * kp

        # Get current orientation from odometry
        current_quat = self.last_odom.pose.pose.orientation
        
        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([
            current_quat.x, current_quat.y, current_quat.z, current_quat.w
        ])
        
        # Apply yaw command
        desired_yaw = yaw + yaw_command
        
        # Convert back to quaternion
        quat = quaternion_from_euler(roll, pitch, desired_yaw)
        
        # Create goal pose
        goal_pose = Pose()
        goal_pose.position = self.last_odom.pose.pose.position
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]
        
        # Publish the goal pose
        self.goal_pose_pub.publish(goal_pose)
        

def main():
    rclpy.init()
    node = CentriodYawTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
