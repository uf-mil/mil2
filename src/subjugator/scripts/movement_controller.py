#!/usr/bin/env python3
"""
movement_controller.py

High-level movement controller for SubjuGator AUV.
Publishes geometry_msgs/Wrench to /cmd_wrench to command pure
translational motion (X/Y/Z) without inducing roll, pitch, or yaw.

The thruster_manager node receives this wrench and uses the
Thruster Allocation Matrix (TAM) pseudoinverse to distribute
forces across the 8 thrusters (FLH, FRH, BLH, BRH, FLV, FRV, BLV, BRV).

Related issue: #470 - better thruster control (torpedo task)
"""

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node


class MovementController(Node):
    def __init__(self):
        super().__init__("movement_controller")

        self.publisher_ = self.create_publisher(Wrench, "/cmd_wrench", 10)

        # Declare parameters for force in each axis (default 0.0)
        self.declare_parameter("force_x", 0.0)  # forward/backward
        self.declare_parameter("force_y", 0.0)  # left/right strafe
        self.declare_parameter("force_z", 0.0)  # up/down (depth)

        self.timer = self.create_timer(0.1, self.publish_wrench)
        self.get_logger().info("Movement controller started. Publishing to /cmd_wrench")

    def publish_wrench(self):
        force_x = self.get_parameter("force_x").get_parameter_value().double_value
        force_y = self.get_parameter("force_y").get_parameter_value().double_value
        force_z = self.get_parameter("force_z").get_parameter_value().double_value

        msg = Wrench()

        # Set desired translational forces
        msg.force.x = force_x  # forward/backward (horizontal thrusters)
        msg.force.y = force_y  # left/right strafe (horizontal thrusters)
        msg.force.z = force_z  # up/down depth     (vertical thrusters)

        # Explicitly zero out all torques to prevent roll/pitch/yaw
        msg.torque.x = 0.0  # no roll
        msg.torque.y = 0.0  # no pitch
        msg.torque.z = 0.0  # no yaw

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published wrench -> X: {force_x:.2f}N, Y: {force_y:.2f}N, Z: {force_z:.2f}N | "
            f"Torques: all zero (no rotation)",
        )


def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down movement controller.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
