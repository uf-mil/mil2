#!/usr/bin/env python3

import json
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

HOST = "127.0.0.1"
PORT = 1234


class SensorBridge(Node):
    def __init__(self):
        super().__init__("infix_sensor_bridge")

        # ROS publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)

        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)

        self.get_logger().info(f"Connecting to sensor stream at {HOST}:{PORT}")

        while True:
            try:
                self.sock = socket.create_connection((HOST, PORT))
                break
            except Exception:
                time.sleep(1)

        self.stream = self.sock.makefile("r")

        self.stream.readline()  # first line is json header. ignore

        self.get_logger().info("Connected to sensor stream")
        self.loop()

    def loop(self):
        while True:
            j = json.loads(self.stream.readline())

            # ---------------------------------------------------------
            # IMU
            # ---------------------------------------------------------
            if imu_packet := j.get("sdgps::interfaces::common::IMUPacket", None):
                acceleration = imu_packet["proper_acceleration"]
                angular_velocity = imu_packet["angular_velocity"]

                msg = Imu()

                # ROS timestamp
                # msg.header.stamp = self.get_clock().now().to_msg()

                # Change this to whatever coordinate frame your IMU uses.
                msg.header.frame_id = "base_link"

                # proper_acceleration is assumed to be m/s^2
                msg.linear_acceleration.x = acceleration[0]
                msg.linear_acceleration.y = acceleration[1]
                msg.linear_acceleration.z = acceleration[2]

                # angular_velocity is assumed to be rad/s
                msg.angular_velocity.x = angular_velocity[0]
                msg.angular_velocity.y = angular_velocity[1]
                msg.angular_velocity.z = angular_velocity[2]

                # covariances from: `sdgps print-config ugi2:`
                msg.orientation_covariance[0] = -1

                msg.linear_acceleration_covariance[0] = 3e-4
                msg.linear_acceleration_covariance[4] = 3e-4
                msg.linear_acceleration_covariance[8] = 4.5e-4

                msg.orientation_covariance[0] = 2e-6
                msg.orientation_covariance[4] = 2e-6
                msg.orientation_covariance[8] = 1.5e-6

                self.imu_pub.publish(msg)

            # Magnetometer
            elif mag_packet := j.get(
                "sdgps::interfaces::common::MagnetometerPacket",
                None,
            ):
                magnetic_field = mag_packet["magnetic_field"]
                msg = MagneticField()

                # msg.header.stamp = self.get_clock().now().to_msg()
                # msg.header.frame_id = "base_link"

                msg.magnetic_field.x = magnetic_field[0]
                msg.magnetic_field.y = magnetic_field[1]
                msg.magnetic_field.z = magnetic_field[2]

                # covariances from: `sdgps print-config ugi2:`
                msg.magnetic_field_covariance[0] = 4e-13
                msg.magnetic_field_covariance[4] = 4e-13
                msg.magnetic_field_covariance[8] = 4e-13

                self.mag_pub.publish(msg)


def main():
    rclpy.init()
    node = SensorBridge()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
