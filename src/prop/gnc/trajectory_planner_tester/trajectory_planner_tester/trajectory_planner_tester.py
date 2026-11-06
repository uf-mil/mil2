#!/usr/bin/env python3
import threading

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path
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
            Path,
            "/plan",
            self._plan_callback,
            reliable_qos,
        )
        self._odom_pub = self.create_publisher(
            Odometry,
            "/odometry/filtered/local",
            reliable_qos,
        )
        self.create_timer(0.05, self._publish_odom)
        self.get_logger().info("TrajectoryDebugSim started.")

    def _plan_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("Received empty path, ignoring.")
            return
        self.get_logger().info(
            f"Received path with {len(msg.poses)} waypoints. " f"Simulating travel...",
        )
        self._schedule_waypoints(list(msg.poses), 0)

    def _schedule_waypoints(self, poses, index):
        if index >= len(poses):
            self.get_logger().info("All waypoints reached.")
            return

        goal = poses[index].pose
        fired = {"done": False}

        def _callback():
            if fired["done"]:
                return
            fired["done"] = True
            with self._lock:
                self._current_pose = goal
            self.get_logger().info(
                f"Arrived at waypoint {index + 1}/{len(poses)} "
                f"({goal.position.x:.2f}, {goal.position.y:.2f}).",
            )
            self._schedule_waypoints(poses, index + 1)

        self.create_timer(self.ARRIVAL_DELAY_SEC, _callback)

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
