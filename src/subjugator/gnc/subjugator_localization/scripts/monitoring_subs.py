#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MonitoringNode(Node):
    def __init__(self):
        super().__init__("monitoring_node")
        self.data = ""
        self.pub_dvl_ = self.create_publisher(Odometry, "monitoring_dvl", 10)
        self.pub_imu_ = self.create_publisher(Imu, "monitoring_imu", 10)

        self.create_subscription(Odometry, "dvl/odom", self.dvl_odom_callback, 10)
        self.create_subscription(Imu, "imu/data", self.imu_data_callback, 10)

        self.imu_accelerationx_array = []
        self.imu_accelerationy_array = []
        self.imu_accelerationz_array = []

    def dvl_odom_callback(self, dmsg: Odometry):
        self.pub_dvl_.publish(dmsg)

    def imu_data_callback(self, imsg: Imu):
        # create running average for relevant measurements:
        # acceleration x
        if len(self.imu_accelerationx_array) <= 3:
            self.imu_accelerationx_array.append(imsg.linear_acceleration.x)
        else:
            imu_accelerationx_avg = (
                self.imu_accelerationx_array[0]
                + self.imu_accelerationx_array[1]
                + self.imu_accelerationx_array[2]
            ) / len(self.imu_accelerationx_array)
            percent_diff = (
                (imsg.linear_acceleration.x - imu_accelerationx_avg)
                / imu_accelerationx_avg
            ) * 100
            if -500 < percent_diff < 500:  # arbitrary large percentage difference
                self.imu_accelerationx_array.append(imsg.linear_acceleration.x)
                del self.imu_accelerationx_array[0]
            else:
                self.get_logger().info("Spike Detected at lin_accel_x.")

        # acceleration y
        if len(self.imu_accelerationy_array) <= 3:
            self.imu_accelerationy_array.append(imsg.linear_acceleration.y)
        else:
            imu_accelerationy_avg = (
                self.imu_accelerationy_array[0]
                + self.imu_accelerationy_array[1]
                + self.imu_accelerationy_array[2]
            ) / len(self.imu_accelerationy_array)
            percent_diff = (
                (imsg.linear_acceleration.y - imu_accelerationy_avg)
                / imu_accelerationy_avg
            ) * 100
            if -500 < percent_diff < 500:  # arbitrary large percentage difference
                self.imu_accelerationy_array.append(imsg.linear_acceleration.y)
                del self.imu_accelerationy_array[0]
            else:
                self.get_logger().info(
                    "Spike Detected a lin_accel_y.",
                )  # tells us where spike occurred.

        # acceleration z
        if len(self.imu_accelerationz_array) <= 3:
            self.imu_accelerationz_array.append(imsg.linear_acceleration.z)
        else:
            imu_accelerationz_avg = (
                self.imu_accelerationz_array[0]
                + self.imu_accelerationz_array[1]
                + self.imu_accelerationz_array[2]
            ) / len(self.imu_accelerationz_array)
            percent_diff = (
                (imsg.linear_acceleration.z - imu_accelerationz_avg)
                / imu_accelerationz_avg
            ) * 100
            if (
                percent_diff < 500 or percent_diff > -500
            ):  # arbitrary large percentage difference
                self.imu_accelerationz_array.append(imsg.linear_acceleration.z)
                del self.imu_accelerationz_array[0]
            else:
                self.get_logger().info("Spike Detected at lin_accel_z.")

        self.pub_imu_.publish(imsg)


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
