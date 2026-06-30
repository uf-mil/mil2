#!/usr/bin/env python3

"""
Simulate sub odometry for testing move_poses.py on a laptop.

Listens to /goal/trajectory and gradually "moves" a simulated position
toward the goal, publishing the result to /odometry/filtered.

Usage:
  python3 mock_odom.py              # default speed 0.3 m/s
  python3 mock_odom.py --speed 0.5  # faster simulation
  python3 mock_odom.py --start 0,0,-1,0  # custom start pose
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import (
    Pose,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MockOdom(Node):
    def __init__(self, start_pose: Pose, speed: float):
        super().__init__("mock_odom")
        self.current = start_pose
        self.goal: Pose | None = None
        self.speed = speed  # m/s

        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.goal_sub = self.create_subscription(
            Pose,
            "/goal/trajectory",
            self._goal_cb,
            1,
        )

        # update at 20 Hz
        self.create_timer(0.05, self._update)
        self.get_logger().info(f"Mock odom started  speed={speed}m/s")

    def _goal_cb(self, msg: Pose) -> None:
        self.goal = msg
        p = msg.position
        self.get_logger().info(f"New goal → x={p.x:.2f}  y={p.y:.2f}  z={p.z:.2f}")

    def _update(self) -> None:
        if self.goal is not None:
            self._step_toward_goal()
        self._publish()

    def _step_toward_goal(self) -> None:
        dx = self.goal.position.x - self.current.position.x
        dy = self.goal.position.y - self.current.position.y
        dz = self.goal.position.z - self.current.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 0.01:
            return

        # move one timestep toward goal (dt=0.05s)
        step = min(self.speed * 0.05, dist)
        scale = step / dist
        self.current.position.x += dx * scale
        self.current.position.y += dy * scale
        self.current.position.z += dz * scale

    def _publish(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose = self.current
        self.odom_pub.publish(msg)


def _parse_start(s: str) -> Pose:
    parts = s.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(f"Expected x,y,z,yaw_deg — got: {s!r}")
    x, y, z, yaw_deg = map(float, parts)
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    yaw_rad = math.radians(yaw_deg)
    p.orientation.w = math.cos(yaw_rad / 2.0)
    p.orientation.z = math.sin(yaw_rad / 2.0)
    return p


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simulate sub odometry for testing move_poses.py",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Simulated movement speed in m/s (default: 0.3)",
    )
    parser.add_argument(
        "--start",
        default="0,0,-1,0",
        metavar="x,y,z,yaw_deg",
        help="Starting pose (default: 0,0,-1,0)",
    )

    args, _ = parser.parse_known_args()
    start_pose = _parse_start(args.start)

    rclpy.init()
    node = MockOdom(start_pose, args.speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
