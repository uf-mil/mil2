#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from subjugator_msgs.msg import SensorSpike, SensorSpikeArray


class MonitoringNode(Node):
    def __init__(self):
        super().__init__("monitoring_node")
        self.data = ""
        self.pub_spike = self.create_publisher(SensorSpikeArray, "spike_monitoring", 10)
        self.array_msg = SensorSpikeArray()

        self.create_subscription(Odometry, "dvl/odom", self.dvl_odom_callback, 10)
        self.create_subscription(Imu, "imu/data", self.imu_data_callback, 10)

        self.imu_accelerationx_array = []
        self.imu_accelerationy_array = []
        self.imu_accelerationz_array = []

        self.dvl_accelerationx_array = []
        self.dvl_accelerationy_array = []
        self.dvl_accelerationz_array = []

        self.in_spike_state_imu_x = False
        self.in_spike_state_imu_y = False
        self.in_spike_state_imu_z = False

        self.in_spike_state_dvl_x = False
        self.in_spike_state_dvl_y = False
        self.in_spike_state_dvl_z = False

    def process_imu(self, value, array, spike_state_attr, axis_name):

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
                    # publish a SensorSpike message with details about the detected spike
                    msg = SensorSpike()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "imu"
                    msg.spike_detected = True
                    msg.sensor_type = axis_name
                    msg.measured_value = float(value)
                    self.array_msg.sensors_status.append(msg)
                    setattr(self, spike_state_attr, True)

    def process_dvl(self, value, array, spike_state_attr, axis_name):
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
                    # publish a SensorSpike message with details about the detected spike
                    msg = SensorSpike()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "dvl"
                    msg.spike_detected = True
                    msg.sensor_type = axis_name
                    msg.measured_value = float(value)
                    self.array_msg.sensors_status.append(msg)
                    setattr(self, spike_state_attr, True)

    def dvl_odom_callback(self, dmsg: Odometry):
        # Process x-velocity
        self.process_dvl(
            dmsg.twist.twist.linear.x,
            self.dvl_accelerationx_array,
            "in_spike_state_dvl_x",
            "dvl_vel_x",
        )

        # Process y-velocity
        self.process_dvl(
            dmsg.twist.twist.linear.y,
            self.dvl_accelerationy_array,
            "in_spike_state_dvl_y",
            "dvl_vel_y",
        )

        # Process z-velocity
        self.process_dvl(
            dmsg.twist.twist.linear.z,
            self.dvl_accelerationz_array,
            "in_spike_state_dvl_z",
            "dvl_vel_z",
        )

        self.pub_spike.publish(self.array_msg)

    def imu_data_callback(self, imsg: Imu):
        # Process x-axis
        self.process_imu(
            imsg.linear_acceleration.x,
            self.imu_accelerationx_array,
            "in_spike_state_imu_x",
            "lin_accel_x",
        )

        # Process y-axis
        self.process_imu(
            imsg.linear_acceleration.y,
            self.imu_accelerationy_array,
            "in_spike_state_imu_y",
            "lin_accel_y",
        )

        # Process z-axis
        self.process_imu(
            imsg.linear_acceleration.z,
            self.imu_accelerationz_array,
            "in_spike_state_imu_z",
            "lin_accel_z",
        )

        self.pub_spike.publish(self.array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
