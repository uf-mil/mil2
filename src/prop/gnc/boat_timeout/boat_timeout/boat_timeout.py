# how to send the emergency coords
"""
ros2 topic pub --once /latlon_goal/coordinate geographic_msgs/msg/GeoPose \
  "{position: {latitude: 29.6481, longitude: -82.3438, altitude: 0.0}}"
"""
# your coords will vary — get them from the map frame like so:
"""
ros2 service call /toLL robot_localization/srv/ToLL "{map_point: {x: 0.0, y: 0.0, z: 0.0}}"
"""

# send the all clear:
"""
ros2 service call /latlon_goal/all_clear std_srvs/srv/Empty {}
"""

# worth noting!!!!
# this doesn't
# 1. shut anything else down <- other missions running might try and fight it :(
# 2. listen to ocs, we still need a node that connects to ocs and publishes the topic automagically

import math
import threading

import rclpy
import tf2_geometry_msgs  # noqa: F401  -- registers PoseStamped transforms
import tf2_ros
from geographic_msgs.msg import GeoPoint, GeoPose
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
    RETURNING = "RETURNING"

    ARRIVAL_THRESHOLD = 0.7  # metres (euclidean)
    DRIFT_THRESHOLD = 0.7    # metres — re-issue goal if drift exceeds this

    def __init__(self):
        super().__init__("latlon_goal_node")

        self._state = self.IDLE
        self._state_lock = threading.Lock()

        self._map_goal_pose: PoseStamped | None = None
        self._map_goal_pose_lock = threading.Lock()

        self._last_published_odom_goal: PoseStamped | None = None
        self._latest_global_odom: Odometry | None = None

        # Snapshot of where the boat was when the emergency goal was first received
        self._return_pose: PoseStamped | None = None

        cb = ReentrantCallbackGroup()

        # --- Publishers ---
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # --- Subscribers ---
        self.create_subscription(
            Odometry,
            "odometry/filtered/global",
            self._odom_cb,
            10,
            callback_group=cb,
        )

        self.create_subscription(
            GeoPose,
            "latlon_goal/coordinate",
            self._coordinate_cb,
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
    # Coordinate topic callback — emergency entry point
    # ------------------------------------------------------------------
    def _coordinate_cb(self, msg: GeoPose):
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        self.get_logger().info(
            f"Emergency coord received via topic: {lat}, {lon}, alt={alt}"
        )
        self.send_latlon_goal(lat, lon, alt)

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------
    def send_latlon_goal(self, lat: float, lon: float, alt: float = 0.0):
        self.get_logger().info(f"New lat/lon goal received: {lat}, {lon}, alt={alt}")

        # Snapshot current position so we can return here on all-clear.
        # Only snapshot if we're not already mid-return (avoids overwriting
        # the original position if someone sends a second emergency goal)
        with self._state_lock:
            if self._state not in (self.RETURNING,):
                self._return_pose = self._current_pose()
                if self._return_pose is not None:
                    self.get_logger().info(
                        f"Return pose saved: "
                        f"({self._return_pose.pose.position.x:.2f}, "
                        f"{self._return_pose.pose.position.y:.2f})"
                    )
                else:
                    self.get_logger().warn(
                        "No odometry available yet — return pose not saved."
                    )
            self._state = self.NAVIGATING

        threading.Thread(
            target=self._navigate_to_latlon,
            args=(lat, lon, alt),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    # Core navigation logic (runs in its own thread)
    # ------------------------------------------------------------------
    def _navigate_to_latlon(self, lat: float, lon: float, alt: float):
        map_pose = self._latlon_to_map(lat, lon, alt)
        if map_pose is None:
            self.get_logger().error("fromLL service failed — aborting goal.")
            with self._state_lock:
                self._state = self.IDLE
            return

        with self._map_goal_pose_lock:
            self._map_goal_pose = map_pose

        if not self._publish_goal_in_odom(map_pose):
            return

        self._arrival_loop()

    # ------------------------------------------------------------------
    # Arrival + drift-correction loop
    # ------------------------------------------------------------------
    def _arrival_loop(self):
        rate = self.create_rate(2)  # 2 Hz

        while rclpy.ok():
            with self._state_lock:
                current_state = self._state
                if current_state not in (self.NAVIGATING, self.RETURNING):
                    return

            current = self._current_pose()
            if current is None:
                rate.sleep()
                continue

            with self._map_goal_pose_lock:
                goal_map = self._map_goal_pose

            if goal_map is None:
                rate.sleep()
                continue

            dist = self._distance_in_map(current, goal_map)
            self.get_logger().debug(f"Distance to goal: {dist:.3f} m")

            if dist <= self.ARRIVAL_THRESHOLD:
                with self._state_lock:
                    if self._state == self.RETURNING:
                        self.get_logger().info(
                            f"Returned to original position (dist={dist:.2f} m). IDLE."
                        )
                        self._state = self.IDLE
                    else:
                        self.get_logger().info(
                            f"Arrived at goal (dist={dist:.2f} m). Entering HOLDING state."
                        )
                        self._state = self.HOLDING
                return

            # Drift correction
            odom_goal = self._map_to_odom(_copy_pose_stamped(goal_map))
            if odom_goal is not None:
                drift = self._distance_between_poses(
                    odom_goal,
                    self._last_published_odom_goal,
                )
                if drift > self.DRIFT_THRESHOLD:
                    self.get_logger().info(
                        f"Odom drift detected ({drift:.2f} m) — re-issuing goal."
                    )
                    self._publish_goal_in_odom(goal_map)

            rate.sleep()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _latlon_to_map(self, lat: float, lon: float, alt: float) -> PoseStamped | None:
        if not self._from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/fromLL service not available.")
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=alt)
        future = self._from_ll_client.call_async(req)

        timeout = 10.0
        start = self.get_clock().now()
        while not future.done():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error("/fromLL timed out.")
                return None
            threading.Event().wait(0.05)

        if future.result() is None:
            self.get_logger().error("/fromLL returned no result.")
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = future.result().map_point
        pose.pose.orientation.w = 1.0
        return pose

    def _map_to_odom(self, map_pose: PoseStamped) -> PoseStamped | None:
        """map_pose is consumed (stamp is overwritten) — pass a copy if you need the original."""
        try:
            map_pose.header.stamp = rclpy.time.Time().to_msg()  # time=0 → latest
            return self._tf_buffer.transform(
                map_pose,
                "odom",
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"TF map→odom failed: {e}")
            return None

    def _publish_goal_in_odom(self, map_pose: PoseStamped) -> bool:
        odom_pose = self._map_to_odom(_copy_pose_stamped(map_pose))
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
            f"({odom_pose.pose.position.x:.2f}, {odom_pose.pose.position.y:.2f}, "
            f"z={odom_pose.pose.position.z:.2f})"
        )
        return True

    def _odom_cb(self, msg: Odometry):
        self._latest_global_odom = msg

    def _current_pose(self) -> PoseStamped | None:
        odom = self._latest_global_odom
        if odom is None:
            return None
        if odom.header.frame_id != "map":
            self.get_logger().warn(
                f"Global odom frame is '{odom.header.frame_id}', expected 'map' — "
                "distance calculations may be wrong!",
                throttle_duration_sec=5.0,
            )
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        return ps

    def _distance_in_map(self, current: PoseStamped, goal: PoseStamped) -> float:
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
            return_pose = self._return_pose

        if return_pose is not None:
            self.get_logger().info(
                f"All-clear received (was {prev}). Returning to "
                f"({return_pose.pose.position.x:.2f}, {return_pose.pose.position.y:.2f})."
            )
            with self._map_goal_pose_lock:
                self._map_goal_pose = _copy_pose_stamped(return_pose)
            with self._state_lock:
                self._state = self.RETURNING
            self._publish_goal_in_odom(return_pose)
            # Spin up the arrival loop for the return leg
            threading.Thread(target=self._arrival_loop, daemon=True).start()
        else:
            self.get_logger().warn(
                f"All-clear received (was {prev}) but no return pose was saved — going IDLE."
            )
            with self._state_lock:
                self._state = self.IDLE
            with self._map_goal_pose_lock:
                self._map_goal_pose = None

        return response


# ------------------------------------------------------------------
# Utility
# ------------------------------------------------------------------
def _copy_pose_stamped(ps: PoseStamped) -> PoseStamped:
    copy = PoseStamped()
    copy.header.frame_id = ps.header.frame_id
    copy.header.stamp.sec = ps.header.stamp.sec
    copy.header.stamp.nanosec = ps.header.stamp.nanosec
    copy.pose.position.x = ps.pose.position.x
    copy.pose.position.y = ps.pose.position.y
    copy.pose.position.z = ps.pose.position.z
    copy.pose.orientation.x = ps.pose.orientation.x
    copy.pose.orientation.y = ps.pose.orientation.y
    copy.pose.orientation.z = ps.pose.orientation.z
    copy.pose.orientation.w = ps.pose.orientation.w
    return copy


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------
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
