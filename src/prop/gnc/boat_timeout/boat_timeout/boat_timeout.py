# le you are pathetic and awkward

# how to send the emergency coords
"""
ros2 topic pub --once /latlon_goal/coordinate geographic_msgs/msg/GeoPose "{pose: {position: {latitude: 29.6481, longitude: -82.3438, altitude: 0.0}}}"
"""
# your cords will vary
# BUT JOE HOW WILL THEY VARY???
# well Joe you pathetic loser they vary with the pool ur at mostly, you can get some Lat Lon cords from the map frame as such:
"""
ros2 service call /toLL robot_localization/srv/ToLL "{map_point: {x: 0.0, y: 0.0, z: 0.0}}"
"""
# soooo yeah

# send the all clear:
"""
ros2 service call /latlon_goal/all_clear std_srvs/srv/Empty {}
"""

# worth noting!!!!
# this doesn't
# 1. shut anything else down <- other missions running might try and fight it :(
# 2. listen to ocs, we still need a node that connects to ocs and publishes the service automagically

import math
import threading

import rclpy
import tf2_geometry_msgs  # noqa: F401  -- registers PoseStamped transforms
import tf2_ros
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_srvs.srv import Empty


class LatLonGoalNode(Node):

    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    HOLDING = "HOLDING"

    ARRIVAL_THRESHOLD = 0.7  # metres (euclidean)
    DRIFT_THRESHOLD = 0.7  # metres — re-issue goal if drift exceeds this

    def __init__(self):
        super().__init__("latlon_goal_node")

        self._state = self.IDLE
        self._state_lock = threading.Lock()

        self._map_goal_pose: PoseStamped | None = (
            None  # goal in map frame (stable reference)
        )

        cb = ReentrantCallbackGroup()

        # --- Publishers ---
        self._goal_pub = self.create_publisher(PoseStamped, "/goal/pose", 10)

        # --- Subscribers ---
        self.create_subscription(
            Odometry,
            "odometry/filtered/global",
            self._odom_cb,
            10,
            callback_group=cb,
        )

        # --- Services (provided) ---
        self.create_service(
            Empty,
            "latlon_goal/all_clear",
            self._all_clear_cb,
            callback_group=cb,
        )

        # --- Service clients ---
        self._from_ll_client = self.create_client(FromLL, "/fromLL", callback_group=cb)

        # --- TF ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info("LatLonGoalNode ready.")

    # ------------------------------------------------------------------
    # Public entry point — call this to command a lat/lon goal
    # ------------------------------------------------------------------
    def send_latlon_goal(self, lat: float, lon: float):
        """
        Interrupt whatever the robot is doing and navigate to (lat, lon).
        This is thread-safe and can be called from a subscriber / action / etc.
        """
        self.get_logger().info(f"New lat/lon goal received: {lat}, {lon}")
        with self._state_lock:
            self._state = self.NAVIGATING

        # Spin off async work so we never block the executor
        threading.Thread(
            target=self._navigate_to_latlon,
            args=(lat, lon),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    # Core navigation logic (runs in its own thread)
    # ------------------------------------------------------------------
    def _navigate_to_latlon(self, lat: float, lon: float):
        # 1. Convert lat/lon → map frame via robot_localization
        map_pose = self._latlon_to_map(lat, lon)
        if map_pose is None:
            self.get_logger().error("fromLL service failed — aborting goal.")
            with self._state_lock:
                self._state = self.IDLE
            return

        self._map_goal_pose = map_pose

        # 2. Transform map → odom and publish goal once
        if not self._publish_goal_in_odom(map_pose):
            return

        # 3. Wait until we arrive (with drift correction loop)
        self._arrival_loop()

    # ------------------------------------------------------------------
    # Arrival + drift-correction loop
    # ------------------------------------------------------------------
    def _arrival_loop(self):
        rate = self.create_rate(2)  # 2 Hz polling — not spamming the planner

        while rclpy.ok():
            with self._state_lock:
                if self._state != self.NAVIGATING:
                    return  # pre-empted or cleared externally

            current = self._current_pose()
            if current is None:
                rate.sleep()
                continue

            goal_map = self._map_goal_pose
            dist = self._distance_in_map(current, goal_map)

            self.get_logger().debug(f"Distance to goal: {dist:.3f} m")

            if dist <= self.ARRIVAL_THRESHOLD:
                self.get_logger().info(
                    f"Arrived at goal (dist={dist:.2f} m). Entering HOLDING state.",
                )
                with self._state_lock:
                    self._state = self.HOLDING
                return

            # Check if odom drift means we should re-issue the goal
            odom_goal = self._map_to_odom(goal_map)
            if odom_goal is not None:
                drift = self._distance_between_poses(
                    odom_goal,
                    self._last_published_odom_goal,
                )
                if drift > self.DRIFT_THRESHOLD:
                    self.get_logger().info(
                        f"Odom drift detected ({drift:.2f} m) — re-issuing goal.",
                    )
                    self._publish_goal_in_odom(goal_map)

            rate.sleep()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _latlon_to_map(self, lat: float, lon: float) -> PoseStamped | None:
        if not self._from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/fromLL service not available.")
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        future = self._from_ll_client.call_async(req)

        # Block this thread (not the executor) until we have a result
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error("/fromLL returned no result.")
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = future.result().map_point
        pose.pose.orientation.w = 1.0  # heading doesn't matter for a 2D positional goal
        return pose

    def _map_to_odom(self, map_pose: PoseStamped) -> PoseStamped | None:
        try:
            # Use latest available transform — fine for 2D slow robot
            map_pose.header.stamp = rclpy.time.Time().to_msg()  # time=0 → latest
            odom_pose = self._tf_buffer.transform(
                map_pose,
                "odom",
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return odom_pose
        except Exception as e:
            self.get_logger().warn(f"TF map→odom failed: {e}")
            return None

    _last_published_odom_goal: PoseStamped | None = None

    def _publish_goal_in_odom(self, map_pose: PoseStamped) -> bool:
        odom_pose = self._map_to_odom(map_pose)
        if odom_pose is None:
            self.get_logger().error("Cannot transform goal to odom frame — aborting.")
            with self._state_lock:
                self._state = self.IDLE
            return False

        odom_pose.header.stamp = self.get_clock().now().to_msg()
        self._goal_pub.publish(odom_pose)
        self._last_published_odom_goal = odom_pose
        self.get_logger().info(
            f"Published goal in odom frame: "
            f"({odom_pose.pose.position.x:.2f}, {odom_pose.pose.position.y:.2f})",
        )
        return True

    # Current robot pose in the map frame (derived from global odom)
    _latest_global_odom: Odometry | None = None

    def _odom_cb(self, msg: Odometry):
        self._latest_global_odom = msg

    def _current_pose(self) -> PoseStamped | None:
        odom = self._latest_global_odom
        if odom is None:
            return None
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        return ps

    def _distance_in_map(self, current: PoseStamped, goal: PoseStamped) -> float:
        # Both should already be in map frame via global odom
        dx = current.pose.position.x - goal.pose.position.x
        dy = current.pose.position.y - goal.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _distance_between_poses(
        self,
        a: PoseStamped | None,
        b: PoseStamped | None,
    ) -> float:
        if a is None or b is None:
            return 0.0
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    # ------------------------------------------------------------------
    # All-clear service handler
    # ------------------------------------------------------------------
    def _all_clear_cb(self, request, response):
        with self._state_lock:
            prev = self._state
            self._state = self.IDLE
            self._map_goal_pose = None
        self.get_logger().info(
            f"All-clear received (was in {prev} state). Resuming normal ops.",
        )
        return response


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LatLonGoalNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
