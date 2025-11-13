#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MonitoringNode(Node):
    def __init__(self):
        super().__init__("monitoring_node")
        self.data = ""
        """ keeping this here for now, will replace with correct message type

        self.pub_dvl_ = self.create_publisher(Odometry, "monitoring_dvl", 10)
        self.pub_imu_ = self.create_publisher(Imu, "monitoring_imu", 10)
        """

        self.create_subscription(Odometry, "dvl/odom", self.dvl_odom_callback, 10)
        self.create_subscription(Imu, "imu/data", self.imu_data_callback, 10)

        self.imu_accelerationx_array = []
        self.imu_accelerationy_array = []
        self.imu_accelerationz_array = []

        self.in_spike_statex = False
        self.in_spike_statey = False
        self.in_spike_statez = False

    def process_acceleration_axis(self, value, array, spike_state_attr, axis_name):

        if len(array) <= 20:
            array.append(value)
        else:
            # Calculate average using sum and abs
            avg = sum(abs(val) for val in array) / len(array)

            # Calculate percent difference
            percent_diff = ((abs(value) - avg) / avg) * 100

            # Check if within threshold
            if -3000 < percent_diff < 3000:
                array.append(value)
                del array[0]
                setattr(self, spike_state_attr, False)
            else:
                if not getattr(self, spike_state_attr):
                    self.get_logger().info(f"Spike Detected at {axis_name}: {value}")
                    setattr(self, spike_state_attr, True)

    def dvl_odom_callback(self, dmsg: Odometry):
        """keeping this here for now, will replace later

        self.pub_dvl_.publish(dmsg)
        """

    def imu_data_callback(self, imsg: Imu):
        # Process x-axis
        self.process_acceleration_axis(
            imsg.linear_acceleration.x,
            self.imu_accelerationx_array,
            "in_spike_statex",
            "lin_accel_x",
        )

        # Process y-axis
        self.process_acceleration_axis(
            imsg.linear_acceleration.y,
            self.imu_accelerationy_array,
            "in_spike_statey",
            "lin_accel_y",
        )

        # Process z-axis
        self.process_acceleration_axis(
            imsg.linear_acceleration.z,
            self.imu_accelerationz_array,
            "in_spike_statez",
            "lin_accel_z",
        )


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
