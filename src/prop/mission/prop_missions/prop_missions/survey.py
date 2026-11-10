#!/usr/bin/env python3
"""
Survey an area: drive back and forth over it, letting the grid planner steer
round whatever is in the way.

  in   /odometry/filtered/global  nav_msgs/Odometry, the global estimate, in map
       /prop/course_boundary      geographic_msgs/GeoPath, latched, from the OCS
       /occupancy_grid            nav_msgs/OccupancyGrid, what the planner steers by
  out  /goal_pose                 geometry_msgs/PoseStamped, in odom, for the planner
       ~/route                    nav_msgs/Path, the whole pattern in map, for RViz
  srv  ~/start                    std_srvs/Trigger, start surveying
       ~/stop                     std_srvs/Trigger, hold where the boat is
  uses /fromLL                    robot_localization, navsat_transform's

area picks what is surveyed:

  course  the course boundary RoboCommand gives the OCS, or the boundary
          parameter if it is set: lanes spacing apart along its longest side,
          kept margin inside it, starting from the end nearest the boat
  here    a rectangle from the boat: length ahead, width to side

Waypoints are kept in map and reached the way boat_timeout reaches a lat/lon:
lat/lon goes to map through /fromLL, each goal is sent in odom through the
map -> odom transform, arrival is judged in map, and a goal is sent again when
odom drifts it more than drift_threshold. A waypoint with an occupied cell
within clearance is skipped, and so is one not reached in goal_timeout.

Settings are read when ~/start is called, so they can be changed in between:

    ros2 param set /survey spacing 8.0
    ros2 service call /survey/start std_srvs/srv/Trigger
"""

import math

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  -- registers PoseStamped transforms
import tf2_ros
from geographic_msgs.msg import GeoPath, GeoPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_localization.srv import FromLL
from std_srvs.srv import SetBool, Trigger

from prop_missions.geo import yaw_of

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SIDES = {"port": 1.0, "starboard": -1.0}
FROM_LL_TIMEOUT = 10.0  # s


def divisions(length, most):
    """
    How many equal parts length needs to keep each at most most long, give or
    take a centimetre, so lat/lon round trips do not add a part.
    """
    return max(1, math.ceil((length - 0.01) / most))


def lawnmower(origin, heading, length, width, spacing, step, side):
    """
    Waypoints (x, y, heading) back and forth over the rectangle that starts at
    origin and runs length along heading and width to side, lanes at most
    spacing apart and waypoints at most step apart. The first lane leaves from
    origin, which is not itself a waypoint.
    """
    along = np.linspace(0.0, length, divisions(length, step) + 1)
    across = np.linspace(0.0, width, divisions(width, spacing) + 1)
    forward = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-forward[1], forward[0]]) * SIDES[side]
    route = []
    for lane, offset in enumerate(across):
        stations = along if lane % 2 == 0 else along[::-1]
        if lane == 0:
            stations = stations[1:]
        lane_heading = heading if lane % 2 == 0 else heading + math.pi
        for s in stations:
            x, y = np.asarray(origin) + s * forward + offset * left
            route.append((float(x), float(y), lane_heading))
    return route


def polygon_lanes(polygon, spacing, step, margin, start):
    """
    Waypoints (x, y, heading) back and forth inside polygon, a list of (x, y)
    vertices that may repeat the first at the end. Lanes run along its longest
    side, at most spacing apart and margin inside it, with waypoints at most
    step apart, and begin at the corner of the pattern nearest start.
    """
    points = np.asarray(polygon, dtype=float)
    if len(points) > 1 and np.allclose(points[0], points[-1]):
        points = points[:-1]
    if len(points) < 3:
        return []
    edges = np.roll(points, -1, axis=0) - points
    longest = edges[np.argmax(np.hypot(edges[:, 0], edges[:, 1]))]
    u = longest / np.hypot(*longest)
    v = np.array([-u[1], u[0]])
    across = points @ v
    low, high = across.min() + margin, across.max() - margin
    if high < low:
        return []
    offsets = np.linspace(low, high, divisions(high - low, spacing) + 1)

    lanes = []
    for c in offsets:
        ts = []
        for a, b in zip(points, np.roll(points, -1, axis=0)):
            da, db = a @ v - c, b @ v - c
            if da == db:
                continue
            if min(da, db) <= 0.0 <= max(da, db):
                ts.append((a + (b - a) * da / (da - db)) @ u)
        if len(ts) < 2:
            continue
        first, last = min(ts) + margin, max(ts) - margin
        if last < first:
            continue
        stations = np.linspace(first, last, divisions(last - first, step) + 1)
        lanes.append([c * v + t * u for t in stations])
    if not lanes:
        return []

    start = np.asarray(start, dtype=float)
    ends = [lanes[0][0], lanes[0][-1], lanes[-1][0], lanes[-1][-1]]
    nearest = int(np.argmin([np.hypot(*(e - start)) for e in ends]))
    if nearest >= 2:
        lanes.reverse()
    reverse = nearest in (1, 3)
    route = []
    for lane in lanes:
        lane = lane[::-1] if reverse else lane
        heading = math.atan2(*(lane[-1] - lane[0])[::-1]) if len(lane) > 1 else 0.0
        route += [(float(p[0]), float(p[1]), heading) for p in lane]
        reverse = not reverse
    return route


class Grid:
    """An OccupancyGrid's cells, to ask whether a point is near an obstacle."""

    def __init__(self, msg):
        info = msg.info
        self.frame = msg.header.frame_id
        self.resolution = info.resolution
        self.origin = (info.origin.position.x, info.origin.position.y)
        self.cells = np.asarray(msg.data, dtype=np.int16).reshape(
            info.height,
            info.width,
        )

    def blocked(self, x, y, clearance, occupied):
        """
        Whether a cell at or above occupied lies within clearance of (x, y).
        Cells off the grid or unknown are not obstacles.
        """
        r = math.ceil(clearance / self.resolution)
        col = math.floor((x - self.origin[0]) / self.resolution)
        row = math.floor((y - self.origin[1]) / self.resolution)
        h, w = self.cells.shape
        rows = slice(max(0, row - r), min(h, row + r + 1))
        cols = slice(max(0, col - r), min(w, col + r + 1))
        patch = self.cells[rows, cols]
        if patch.size == 0:
            return False
        ys, xs = np.nonzero(patch >= occupied)
        cx = self.origin[0] + (xs + cols.start + 0.5) * self.resolution
        cy = self.origin[1] + (ys + rows.start + 0.5) * self.resolution
        return bool(np.any(np.hypot(cx - x, cy - y) <= clearance))


def pose_stamped(frame, x, y, heading=0.0):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(heading / 2)
    pose.pose.orientation.w = math.cos(heading / 2)
    return pose


class Survey(Node):
    def __init__(self):
        super().__init__("survey")
        p = self.declare_parameter
        for name, default in (
            ("area", "course"),
            ("boundary", [0.0, 0.0]),
            ("margin", 5.0),
            ("length", 30.0),
            ("width", 20.0),
            ("side", "starboard"),
            ("spacing", 10.0),
            ("step", 5.0),
            ("clearance", 1.5),
            ("occupied", 60),
            ("reach", 1.5),
            ("drift_threshold", 0.7),
            ("goal_timeout", 60.0),
            ("return_home", False),
            ("map_frame", "map"),
            ("odom_frame", "odom"),
        ):
            p(name, default)

        self.pose = None  # (x, y, heading) in map
        self.grid = None
        self.course = []  # (lat, lon) from the OCS
        self.route = []  # waypoints left, (x, y, heading) in map
        self.home = None
        self.active = False
        self.converting = None  # futures from /fromLL, while turning lat/lon into map
        self.goal_sent = None
        self.last_goal = None  # the odom goal last published
        self.reached = self.skipped = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.route_pub = self.create_publisher(Path, "~/route", LATCHED)
        self.planner = self.create_client(SetBool, "/planner/enable")
        self.from_ll = self.create_client(FromLL, "/fromLL")
        self.create_subscription(
            Odometry,
            "/odometry/filtered/global",
            self.on_odometry,
            10,
        )
        self.create_subscription(
            GeoPath,
            "/prop/course_boundary",
            self.on_course,
            LATCHED,
        )
        self.create_subscription(
            OccupancyGrid,
            "/occupancy_grid",
            lambda msg: setattr(self, "grid", Grid(msg)),
            LATCHED,
        )
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_timer(0.5, self.step)
        self.get_logger().info("ready; call ~/start to survey")

    def param(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odometry(self, msg):
        if msg.header.frame_id != self.param("map_frame"):
            self.get_logger().warn(
                f"global odometry is in '{msg.header.frame_id}', expected "
                f"'{self.param('map_frame')}'",
                throttle_duration_sec=5.0,
            )
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def on_course(self, msg):
        self.course = [
            (g.pose.position.latitude, g.pose.position.longitude) for g in msg.poses
        ]
        self.get_logger().info(
            f"course boundary from the OCS: {len(self.course)} corners",
        )

    # ----- starting and stopping -----

    def on_start(self, _, response):
        if self.pose is None:
            response.message = "no global odometry yet; is localization started?"
            return response
        self.reached = self.skipped = 0
        self.home = self.pose
        if self.param("area") == "here":
            side = self.param("side")
            if side not in SIDES:
                response.message = f"side must be port or starboard, not {side}"
                return response
            route = lawnmower(
                self.pose[:2],
                self.pose[2],
                self.param("length"),
                self.param("width"),
                self.param("spacing"),
                self.param("step"),
                side,
            )
            self.begin(route)
            response.success = True
            response.message = f"surveying here, {len(route)} waypoints"
            return response
        if self.param("area") != "course":
            response.message = f"area must be course or here, not {self.param('area')}"
            return response

        boundary = list(self.param("boundary"))
        corners = (
            list(zip(boundary[::2], boundary[1::2]))
            if len(boundary) >= 6
            else self.course
        )
        if len(corners) < 3:
            response.message = (
                "no course boundary: none from the OCS, and boundary is not set"
            )
            return response
        if not self.from_ll.service_is_ready():
            response.message = "/fromLL is not up; is localization running?"
            return response
        self.converting = (
            self.now(),
            [
                self.from_ll.call_async(
                    FromLL.Request(ll_point=GeoPoint(latitude=lat, longitude=lon)),
                )
                for lat, lon in corners
            ],
        )
        response.success = True
        response.message = (
            f"turning {len(corners)} corners into map; starts once they are"
        )
        self.get_logger().info(response.message)
        return response

    def converted(self):
        """Build the course route once /fromLL has answered for every corner."""
        since, futures = self.converting
        if not all(f.done() for f in futures):
            if self.now() - since > FROM_LL_TIMEOUT:
                self.get_logger().error("/fromLL timed out; survey not started")
                self.converting = None
            return
        self.converting = None
        if any(f.result() is None for f in futures):
            self.get_logger().error("/fromLL returned no result; survey not started")
            return
        polygon = [(f.result().map_point.x, f.result().map_point.y) for f in futures]
        route = polygon_lanes(
            polygon,
            self.param("spacing"),
            self.param("step"),
            self.param("margin"),
            self.pose[:2],
        )
        if not route:
            self.get_logger().error(
                "the course is too small for margin; survey not started",
            )
            return
        self.begin(route)

    def begin(self, route):
        self.route = route
        self.goal_sent = None
        self.last_goal = None
        self.active = True
        self.publish_route()
        if not self.planner.service_is_ready():
            self.get_logger().warn("/planner/enable is not up; carrying on without it")
        self.planner.call_async(SetBool.Request(data=True))
        self.get_logger().info(f"surveying: {len(route)} waypoints")

    def on_stop(self, _, response):
        self.active = False
        self.converting = None
        self.route = []
        if self.pose is not None:
            self.publish_goal(*self.pose)
        response.success = True
        response.message = "stopped, holding here"
        self.get_logger().info(response.message)
        return response

    # ----- following the route -----

    def step(self):
        if self.converting is not None:
            self.converted()
            return
        if not self.active or self.pose is None:
            return
        if not self.route:
            self.finish()
            return
        x, y, heading = self.route[0]
        if self.goal_sent is None:
            odom = self.to_odom(x, y, heading)
            if odom is None:
                return
            if self.in_obstacle(odom):
                self.get_logger().info(
                    f"({x:.1f}, {y:.1f}) is in an obstacle; skipping it",
                )
                self.next(reached=False)
                return
            self.publish(odom)
            return
        if math.dist((x, y), self.pose[:2]) <= self.param("reach"):
            self.next(reached=True)
            return
        if self.now() - self.goal_sent > self.param("goal_timeout"):
            self.get_logger().warn(f"could not reach ({x:.1f}, {y:.1f}); skipping it")
            self.next(reached=False)
            return
        odom = self.to_odom(x, y, heading)
        if odom is None:
            return
        if self.in_obstacle(odom):
            self.get_logger().info(
                f"({x:.1f}, {y:.1f}) is now in an obstacle; skipping it",
            )
            self.next(reached=False)
            return
        drift = math.dist(
            (odom.pose.position.x, odom.pose.position.y),
            (self.last_goal.pose.position.x, self.last_goal.pose.position.y),
        )
        if drift > self.param("drift_threshold"):
            self.get_logger().info(f"odom drift ({drift:.2f} m); re-issuing goal")
            self.publish(odom)

    def next(self, reached):
        self.route.pop(0)
        self.goal_sent = None
        if reached:
            self.reached += 1
        else:
            self.skipped += 1

    def finish(self):
        if self.home is None:
            self.get_logger().info("back at the start")
            self.active = False
            return
        self.get_logger().info(
            f"survey done: {self.reached} waypoints reached, {self.skipped} skipped",
        )
        if self.param("return_home"):
            self.get_logger().info("going back to the start")
            self.route = [self.home]
            self.home = None
            return
        self.active = False

    def in_obstacle(self, odom):
        if self.grid is None or self.grid.frame != odom.header.frame_id:
            return False
        return self.grid.blocked(
            odom.pose.position.x,
            odom.pose.position.y,
            self.param("clearance"),
            self.param("occupied"),
        )

    def to_odom(self, x, y, heading=0.0):
        """A map point as a goal in odom, or None without the transform."""
        pose = pose_stamped(self.param("map_frame"), x, y, heading)
        pose.header.stamp = Time().to_msg()  # the latest transform
        try:
            return self.tf_buffer.transform(
                pose,
                self.param("odom_frame"),
                timeout=Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF map -> odom failed: {e}",
                throttle_duration_sec=5.0,
            )
            return None

    def publish(self, odom):
        odom.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(odom)
        self.last_goal = odom
        if self.goal_sent is None:
            self.goal_sent = self.now()

    def publish_goal(self, x, y, heading):
        odom = self.to_odom(x, y, heading)
        if odom is not None:
            self.publish(odom)

    def publish_route(self):
        path = Path()
        path.header.frame_id = self.param("map_frame")
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, heading in self.route:
            pose = pose_stamped(path.header.frame_id, x, y, heading)
            pose.header = path.header
            path.poses.append(pose)
        self.route_pub.publish(path)


def main():
    rclpy.init()
    node = Survey()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
