#!/usr/bin/env python3
"""
Puts covariances back onto the simulated IMU and GPS.

    in   sim/imu, sim/gps_raw   straight off the Gazebo bridge
    out  imu, gps_raw           the same messages, with covariance filled in

The Gazebo sensor messages have nowhere to carry covariance, so ros_gz_bridge
hands over matrices of zeros and the EKFs then treat every measurement as
perfect - which makes the estimate chase the sensor noise instead of filtering
it. The values below are the noise the sensors are configured with in
models/prop/model.sdf; keep the two in step.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

GYRO_STDDEV = 0.004  # rad/s
ACCEL_STDDEV = 0.03  # m/s^2
ROLL_PITCH_STDDEV = math.radians(0.5)
YAW_STDDEV = math.radians(2.0)

GPS_HORIZONTAL_STDDEV = 0.02  # m
GPS_VERTICAL_STDDEV = 0.06  # m


class SimSensors(Node):
    def __init__(self):
        super().__init__("sim_sensors")

        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "gps_raw", 10)

        self.create_subscription(Imu, "sim/imu", self.imu_cb, 10)
        self.create_subscription(NavSatFix, "sim/gps_raw", self.gps_cb, 10)

    def imu_cb(self, imu: Imu) -> None:
        imu.orientation_covariance[0] = ROLL_PITCH_STDDEV**2
        imu.orientation_covariance[4] = ROLL_PITCH_STDDEV**2
        imu.orientation_covariance[8] = YAW_STDDEV**2
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = GYRO_STDDEV**2
            imu.linear_acceleration_covariance[i] = ACCEL_STDDEV**2
        self.imu_pub.publish(imu)

    def gps_cb(self, fix: NavSatFix) -> None:
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance[0] = GPS_HORIZONTAL_STDDEV**2
        fix.position_covariance[4] = GPS_HORIZONTAL_STDDEV**2
        fix.position_covariance[8] = GPS_VERTICAL_STDDEV**2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(fix)


def main():
    rclpy.init()
    node = SimSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
