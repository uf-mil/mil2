#!/usr/bin/env python3
"""Redundancy check node for comparing DVL and IMU sensor data."""
import rclpy
import rclpy.node
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RedundancyCheckNode(rclpy.node.Node):
    """Node to check redundancy between DVL and IMU sensors."""

    def __init__(self):
        """Initialize the redundancy check node."""
        super().__init__("redundancy_check_node")

        self.imu_subscriber = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10,
        )

        self.dvl_subscriber = self.create_subscription(
            Odometry,
            "/dvl/odom",
            self.dvl_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.imu_data = None
        self.dvl_data = None
        self.previous_dvl_time = None
        self.previous_dvl_vel = None

        self.diff_threshold = 10  # threshold val for redundancy check

        self.get_logger().info("Redundancy Check Node has been started.")

    def imu_callback(self, msg):
        """Handle incoming IMU messages."""
        self.imu_data = msg

    def dvl_callback(self, msg):
        """Handle incoming DVL messages."""
        self.dvl_data = msg

        self.check_redundancy()

    def check_redundancy(self):
        """Check redundancy between DVL and IMU data."""
        # Main redundancy check logic:
        if self.imu_data is None or self.dvl_data is None:
            return

        # Calculate current time from DVL data timestamp
        current_time = rclpy.time.Time.from_msg(self.dvl_data.header.stamp)

        if self.previous_dvl_time is None:
            self.previous_dvl_time = current_time
            self.previous_dvl_vel = self.dvl_data.twist.twist.linear
            return

        dt = (current_time - self.previous_dvl_time).nanoseconds / 1e9

        if dt <= 0:
            self.get_logger().warn(
                "dt is neg. or zero, skipping redundancy check.",
            )
            return

        try:

            # do transforms now
            # Transform IMU linear acceleration to base_link
            imu_lin_acc = Vector3Stamped()
            imu_lin_acc.header.frame_id = "imu_link"
            imu_lin_acc.header.stamp = rclpy.time.Time().to_msg()
            imu_lin_acc.vector = self.imu_data.linear_acceleration

            imu_lin_acc_transformed = self.tf_buffer.transform(
                imu_lin_acc,
                "base_link",
                timeout=Duration(seconds=1.0),
            )

            # Transform DVL linear velocity to base_link
            dvl_linear_vel = Vector3Stamped()
            dvl_linear_vel.header.frame_id = "odom"
            dvl_linear_vel.header.stamp = rclpy.time.Time().to_msg()
            dvl_linear_vel.vector = self.dvl_data.twist.twist.linear

            dvl_vel_transformed = self.tf_buffer.transform(
                dvl_linear_vel,
                "base_link",
                timeout=Duration(seconds=1.0),
            )

        except TransformException as ex:
            self.get_logger().error(f"Transform error: {ex}")
            return

        imu_linear_acc = imu_lin_acc_transformed.vector
        dvl_linear_vel = dvl_vel_transformed.vector

        # Calculate the derivate of the DVL linear x,y,z velocities
        dvl_acceleration = [
            (dvl_linear_vel.x - self.previous_dvl_vel.x) / dt,
            (dvl_linear_vel.y - self.previous_dvl_vel.y) / dt,
            (dvl_linear_vel.z - self.previous_dvl_vel.z) / dt,
        ]

        # Compare DVL acceleration with IMU linear acceleration
        diff_x = abs(dvl_acceleration[0] - imu_linear_acc.x)
        diff_y = abs(dvl_acceleration[1] - imu_linear_acc.y)
        diff_z = abs(dvl_acceleration[2] - imu_linear_acc.z)

        if (
            diff_x > self.diff_threshold
            or diff_y > self.diff_threshold
            or diff_z > self.diff_threshold
        ):
            self.get_logger().warn(
                f"Redundancy check failed: Differences - "
                f"X: {diff_x}, Y: {diff_y}, Z: {diff_z}",
            )

        self.previous_dvl_time = current_time
        self.previous_dvl_vel = dvl_linear_vel


def main(args=None):
    """Run the redundancy check node."""
    rclpy.init(args=args)
    node = RedundancyCheckNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
