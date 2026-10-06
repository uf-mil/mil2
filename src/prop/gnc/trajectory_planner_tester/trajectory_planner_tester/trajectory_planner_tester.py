#!/usr/bin/env python3

import threading

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class TrajectoryDebugSim(Node):

    ARRIVAL_DELAY_SEC = 3.0

    def __init__(self):
        super().__init__("trajectory_debug_sim")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._current_pose = Pose()
        self._lock = threading.Lock()

        self.create_subscription(
            Pose,
            "/goal_pose",
            self._goal_pose_callback,
            reliable_qos,
        )

        self._odom_pub = self.create_publisher(
            Odometry,
            "/odometry/filtered/local",
            reliable_qos,
        )

        # Publish current pose at a steady rate so the follower node
        # always has fresh odometry to work with
        self.create_timer(0.05, self._publish_odom)

        self.get_logger().info("TrajectoryDebugSim started.")

    def _goal_pose_callback(self, msg: Pose) -> None:
        self.get_logger().info(
            f"Goal received ({msg.position.x:.2f}, {msg.position.y:.2f}). "
            f"Simulating {self.ARRIVAL_DELAY_SEC}s travel time...",
        )
        # Fire a one-shot timer — when it fires, snap the robot to the goal
        self.create_timer(
            self.ARRIVAL_DELAY_SEC,
            self._make_arrival_callback(msg),
        )

    def _make_arrival_callback(self, goal: Pose):
        fired = {"done": False}

        def _callback():
            if fired["done"]:
                return
            fired["done"] = True
            with self._lock:
                self._current_pose = goal
            self.get_logger().info(
                f"Arrived at ({goal.position.x:.2f}, {goal.position.y:.2f}).",
            )

        return _callback

    def _publish_odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        with self._lock:
            msg.pose.pose = self._current_pose
        self._odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryDebugSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
