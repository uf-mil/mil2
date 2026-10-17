#!/usr/bin/env python3

import math
import threading
from collections import deque
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a ROS2 Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q) -> float:
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_difference(a: float, b: float) -> float:
    """Shortest signed angular difference from b to a, in [-pi, pi]."""
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class TrajectoryFollower(Node):

    # ------------------------------------------------------------------ #
    #  Tunable parameters
    # ------------------------------------------------------------------ #
    GOAL_XY_TOLERANCE = 0.5  # metres  consider a waypoint reached
    GOAL_YAW_TOLERANCE = 0.15  # radians  consider heading aligned
    PUBLISH_RATE_HZ = 10.0  # how often we re-publish the current goal

    def __init__(self):
        super().__init__("trajectory_follower")

        # ---- state ---------------------------------------------------- #
        self._lock = threading.Lock()

        # Queue of Path messages waiting to be executed (FIFO)
        self._path_queue: deque[Path] = deque()

        # Flat ordered list of Pose goals extracted from the current path
        self._waypoints: list[Pose] = []
        self._waypoint_index: int = 0

        # Phase per waypoint: "rotate" → face the next waypoint first,
        #                     "drive"  → move to the waypoint
        self._phase: str = "idle"  # "idle" | "rotate" | "drive"

        # Heading to finish on (None: any), and the one held on the final
        # waypoint (None: not holding)
        self._final_yaw: float | None = None

        # Latest odometry in the odom frame (used for progress checks)
        self._odom_local: Odometry | None = None
        self._odom_global: Odometry | None = None

        # ---- QoS ------------------------------------------------------ #
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ---- Subscribers ---------------------------------------------- #
        self.create_subscription(
            Path,
            "/plan",
            self._trajectory_callback,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered/local",
            self._local_odom_callback,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered/global",
            self._global_odom_callback,
            reliable_qos,
        )

        # ---- Publisher ------------------------------------------------ #
        self._goal_pub = self.create_publisher(Pose, "/goal/trajectory", reliable_qos)

        # ---- Control loop timer --------------------------------------- #
        self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._control_loop)

        self.get_logger().info("TrajectoryFollower node started.")

    # ================================================================== #
    #  Callbacks
    # ================================================================== #
    def _publish_goal(self, x: float, y: float, yaw: float) -> None:
        """Publish a goal pose, but only if it differs from the last one sent."""
        KEY_ROUND = 2  # round to cm / ~0.01 rad to avoid float noise
        key = (round(x, KEY_ROUND), round(y, KEY_ROUND), round(yaw, KEY_ROUND))
        if key == getattr(self, "_last_goal_key", None):
            return
        self._last_goal_key = key
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = 0.0
        goal.orientation = quaternion_from_yaw(yaw)
        self._goal_pub.publish(goal)
        self.get_logger().info(
            f"Published goal ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)",
        )

    def _trajectory_callback(self, msg: Path) -> None:
        """A new path replaces the current one: the planner only republishes
        when the route has changed, so the old one is stale."""
        with self._lock:
            self._path_queue.clear()
            self._path_queue.append(deepcopy(msg))
            self._phase = "idle"
            self.get_logger().info(
                f"Received path with {len(msg.poses)} poses. "
                f"Queue depth: {len(self._path_queue)}",
            )

    def _local_odom_callback(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_local = msg

    def _global_odom_callback(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_global = msg

    # ================================================================== #
    #  Control loop
    # ================================================================== #

    def _control_loop(self) -> None:
        with self._lock:
            self._step()

    def _step(self) -> None:
        """
        State machine (called at PUBLISH_RATE_HZ):

          idle
            └─ new path in queue → load waypoints → rotate (face wp[0])
          rotate
            └─ heading aligned   → drive
          drive
            └─ position reached  → next waypoint → rotate
                                 → final waypoint    → hold station on it
        """
        # ---- Nothing to do -------------------------------------------- #
        if self._phase == "idle":
            if not self._path_queue:
                return
            self._load_next_path()

        # ---- Need odometry to proceed --------------------------------- #
        if self._odom_local is None:
            return

        robot_pose = self._odom_local.pose.pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)

        if self._waypoint_index >= len(self._waypoints):
            self._finish_path()
            return

        target: Pose = self._waypoints[self._waypoint_index]

        # ---- Desired heading toward the current waypoint -------------- #
        dx = target.position.x - robot_x
        dy = target.position.y - robot_y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)

        # ---- ROTATE phase --------------------------------------------- #
        if self._phase == "rotate":
            heading_err = abs(angle_difference(desired_yaw, robot_yaw))
            if heading_err <= self.GOAL_YAW_TOLERANCE:
                self.get_logger().debug(
                    f"Heading aligned ({math.degrees(desired_yaw):.1f}°). "
                    f"Switching to drive phase.",
                )
                self._phase = "drive"
                self._publish_goal(robot_x, robot_y, desired_yaw)
            else:
                self._publish_goal(robot_x, robot_y, desired_yaw)
            return

        # ---- DRIVE phase ---------------------------------------------- #
        if self._phase == "drive":
            final = self._waypoint_index == len(self._waypoints) - 1

            if final and dist <= self.GOAL_XY_TOLERANCE:
                hold_yaw = self._final_yaw if self._final_yaw is not None else robot_yaw
                self._publish_goal(target.position.x, target.position.y, hold_yaw)
                yaw_err = abs(angle_difference(hold_yaw, robot_yaw))
                if yaw_err <= self.GOAL_YAW_TOLERANCE:
                    self._finish_path()
                return
            else:
                desired_yaw = robot_yaw + math.remainder(
                    desired_yaw - robot_yaw,
                    math.pi,
                )
                self._publish_goal(target.position.x, target.position.y, desired_yaw)

            if not final and dist <= self.GOAL_XY_TOLERANCE:
                self.get_logger().info(
                    f"Reached waypoint {self._waypoint_index + 1}/"
                    f"{len(self._waypoints)}.",
                )
                self._waypoint_index += 1
                self._phase = "rotate"

    # ================================================================== #
    #  Helpers
    # ================================================================== #

    def _load_next_path(self) -> None:
        """Pop the next path from the queue and unpack its waypoints."""
        path: Path = self._path_queue.popleft()

        self._waypoints = [deepcopy(ps.pose) for ps in path.poses if path.poses]
        self._waypoint_index = 0

        # Only the last pose's orientation means anything; a zero quaternion
        # asks for no particular heading.
        q = path.poses[-1].pose.orientation if path.poses else Quaternion()
        has_heading = math.hypot(q.x, q.y, q.z, q.w) > 1e-6
        self._final_yaw = yaw_from_quaternion(q) if has_heading else None

        if not self._waypoints:
            self.get_logger().warn("Received empty path — skipping.")
            self._phase = "idle"
            return

        self.get_logger().info(
            f"Executing path with {len(self._waypoints)} waypoints. "
            f"{len(self._path_queue)} path(s) still queued.",
        )
        self._phase = "rotate"

    def _finish_path(self) -> None:
        """Called when all waypoints in the current path have been reached."""
        self.get_logger().info("Path complete.")
        self._waypoints = []
        self._waypoint_index = 0

        if self._path_queue:
            self._load_next_path()
        else:
            self._phase = "idle"
            self.get_logger().info("All paths complete. Idling.")


# ======================================================================= #
#  Entry point
# ======================================================================= #


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
