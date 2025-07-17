import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from subjugator_msgs.msg import Centroid
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class CentroidYawTracker(Node):
    def __init__(self):
        super().__init__("centroid_yaw_tracker")

        self.odom_sub_ = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_cb,
            10,
        )
        self.last_odom: Odometry = Odometry()

        self.centroid_sub_ = self.create_subscription(
            Centroid,
            "centroids/green",
            self.centroid_cb,
            10,
        )
        self.last_centroid: Centroid = Centroid()
        self.last_centroid.image_width = 640

        self.goal_pose_pub = self.create_publisher(Pose, "goal_pose", 10)

        self.timer = self.create_timer(1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def centroid_cb(self, msg: Centroid):
        self.last_centroid = msg

    def control_loop(self):
        x_error_pixels = (
            self.last_centroid.image_width / 2 - self.last_centroid.centroid_x
        )
        kp = 2.5 / (2 * self.last_centroid.image_width)

        print(x_error_pixels)
        yaw_command = -x_error_pixels * kp

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
        goal_pose.position = self.last_odom.pose.pose.position
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]

        # Publish the goal pose
        self.goal_pose_pub.publish(goal_pose)


def main():
    rclpy.init()
    node = CentroidYawTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
