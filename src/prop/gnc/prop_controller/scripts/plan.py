#!/usr/bin/env python3
"""
Send the boat a plan from the terminal.

    ros2 run prop_controller plan.py square
    ros2 run prop_controller plan.py 20,0 20,20 0,20 0,0
    ros2 run prop_controller plan.py --relative 10,0 0,10
    ros2 run prop_controller plan.py --list
    ros2 run prop_controller plan.py --stop

Publishes nav_msgs/Path on plan, which is the topic guidance follows and the
one the mission node publishes at startup, so this is the same thing arriving
by hand. A new plan replaces whatever the boat was doing. --stop sends an empty
one, after which guidance stops commanding and the thruster manager drops
thrust a second later.

Waypoints are map frame x, y in metres, x east and y north of wherever
localization took its datum, which is where the boat was when it came up.
--relative reads them in the body frame instead: x ahead, y to port, from where
the boat is now and stepping from each waypoint to the next.

--heading says which way to be pointing at the end, in degrees, 0 east and 90
north, and overrides a mission file's own final_heading. Under --relative it is
read from the boat's current heading, like the waypoints. Without a heading
from either the plan finishes on whatever the boat arrives with, because
guidance only holds a heading when it is asked for one. Nothing constrains the
heading at the waypoints in between, which are driven through rather than
stopped on.

The plan is latched, so this waits for guidance to subscribe, publishes, and
exits; guidance keeps the waypoints from there. --hold keeps the publisher
alive instead, for anything that subscribes later.
"""

import argparse
import math
import re
import sys
import time
from pathlib import Path as FilePath

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy import init, shutdown, spin, spin_once
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from yaml import safe_load

# An x,y pair, including negative ones. These are pulled out of the arguments
# before argparse sees them: it reads a leading minus as an option flag, so
# "-1,0" would come back as an unrecognized argument rather than a waypoint.
COORDINATE = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+),[+-]?(\d+\.?\d*|\.\d+)$")

# Long enough for a controller that is still starting up, short enough that a
# typo in a topic name reads as a warning rather than a hang.
SUBSCRIBER_TIMEOUT = 10.0
ODOM_TIMEOUT = 10.0


def missions_dir():
    return FilePath(get_package_share_directory("prop_controller")) / "config/missions"


def mission_names():
    return sorted(path.stem for path in missions_dir().glob("*.yaml"))


def known_missions():
    return ", ".join(mission_names()) or "(none)"


def named_mission(name):
    """Read config/missions/<name>.yaml as (waypoints, finishing heading)."""
    path = missions_dir() / f"{name}.yaml"
    try:
        document = safe_load(path.read_text())
    except FileNotFoundError:
        raise SystemExit(
            f"no mission named {name!r}. Known missions: {known_missions()}",
        ) from None

    # Mission files are parameter files, so the waypoints sit under a node name
    # wildcard: {'/**': {'ros__parameters': {'waypoints': [...]}}}. These are
    # the keys the mission node reads, so a mission runs the same either way.
    for entry in (document or {}).values():
        params = (entry or {}).get("ros__parameters", {})
        flat = params.get("waypoints")
        if flat:
            if len(flat) % 2 != 0:
                raise SystemExit(f"{path} has an odd number of values, expected x, y")
            points = [
                (float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)
            ]
            heading = params.get("final_heading")
            return points, None if heading is None else math.radians(float(heading))
    raise SystemExit(f"{path} has no waypoints")


def parse_waypoints(tokens):
    """Turn 'x,y' arguments into pairs."""
    points = []
    for token in tokens:
        try:
            x, y = (float(part) for part in token.split(","))
        except ValueError:
            raise SystemExit(f"{token!r} is not an x,y pair, e.g. 20,0") from None
        points.append((x, y))
    return points


def yaw_to_quaternion(yaw):
    """Rotation about z, as (z, w). The other two are zero for a flat turn."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def to_map(points, position, heading):
    """Body frame x ahead, y to port, laid end to end from where the boat is."""
    x, y = position
    out = []
    for ahead, port in points:
        x += ahead * math.cos(heading) - port * math.sin(heading)
        y += ahead * math.sin(heading) + port * math.cos(heading)
        out.append((x, y))
    return out


class Plan(Node):
    def __init__(self, frame):
        super().__init__("plan")

        self.frame = frame
        self.odom = None

        # Guidance subscribes transient local, so match it: a plan sent before
        # guidance is up still reaches it, as long as this node is still alive.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, "plan", latched)
        self.create_subscription(Odometry, "odometry/filtered/global", self.odom_cb, 10)

    def odom_cb(self, odom):
        self.odom = odom

    def wait(self, until, timeout):
        deadline = time.monotonic() + timeout
        while not until() and time.monotonic() < deadline:
            spin_once(self, timeout_sec=0.1)
        return until()

    def pose_now(self):
        if not self.wait(lambda: self.odom is not None, ODOM_TIMEOUT):
            raise SystemExit(
                f"no odometry/filtered/global after {ODOM_TIMEOUT:.0f}s, so there "
                "is nothing to be relative to. Is prop_localization running?",
            )
        pose = self.odom.pose.pose
        q = pose.orientation
        heading = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (pose.position.x, pose.position.y), heading

    def send(self, waypoints, heading=None):
        subscribed = self.wait(
            lambda: self.publisher.get_subscription_count() > 0,
            SUBSCRIBER_TIMEOUT,
        )
        if not subscribed:
            self.get_logger().warn(
                "nothing is subscribed to plan; sending it anyway, but guidance "
                "is probably not running. Launch the sim with control:=true.",
            )

        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # Guidance reads the last pose's orientation and nothing else. A zero
        # quaternion is not a rotation, which is how it says "any heading".
        if path.poses:
            last = path.poses[-1].pose.orientation
            if heading is None:
                last.w = 0.0
            else:
                last.z, last.w = yaw_to_quaternion(heading)

        self.publisher.publish(path)
        if not waypoints:
            self.get_logger().info("sent an empty plan, the boat should stop")
            return
        legs = " -> ".join(f"({x:.1f}, {y:.1f})" for x, y in waypoints)
        finish = (
            "any heading" if heading is None else f"{math.degrees(heading):.0f} degrees"
        )
        self.get_logger().info(
            f"sent {len(waypoints)} waypoints: {legs}, finishing on {finish}",
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "mission",
        nargs="?",
        help="a mission name from config/missions. Give x,y waypoints instead "
        "to fly a plan that is not in a file",
    )
    parser.add_argument(
        "--relative",
        action="store_true",
        help="read the waypoints in the boat's body frame, from where it is now",
    )
    parser.add_argument(
        "--heading",
        type=float,
        metavar="DEG",
        help="heading to finish on, degrees, 0 east and 90 north. Relative to "
        "the boat's current heading under --relative. Omit to finish on "
        "whatever heading the boat arrives with",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="send an empty plan, which stops the boat",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="list the missions in config/missions and exit",
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="stay alive holding the plan latched, instead of exiting",
    )
    parser.add_argument(
        "--frame",
        default="map",
        help="frame the waypoints are in (default: map)",
    )
    points = [token for token in argv if COORDINATE.match(token)]
    args = parser.parse_args([token for token in argv if token not in points])
    args.waypoints = points
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if args.list:
        print("\n".join(mission_names()) or f"no missions in {missions_dir()}")
        return

    if args.mission and args.waypoints:
        raise SystemExit("give a mission name or x,y waypoints, not both")

    mission_heading = None
    if args.stop:
        waypoints = []
    elif args.waypoints:
        waypoints = parse_waypoints(args.waypoints)
    elif args.mission:
        waypoints, mission_heading = named_mission(args.mission)
    else:
        raise SystemExit(
            "give a mission name or some x,y waypoints. "
            f"Known missions: {known_missions()}",
        )

    # --heading wins over the one in the mission file.
    goal_heading = (
        mission_heading if args.heading is None else math.radians(args.heading)
    )

    init()
    node = Plan(args.frame)
    try:
        if args.relative and waypoints:
            position, heading = node.pose_now()
            waypoints = to_map(waypoints, position, heading)
            # --relative means everything is read from where the boat is,
            # the finishing heading included.
            if goal_heading is not None:
                goal_heading = math.remainder(goal_heading + heading, 2.0 * math.pi)
        node.send(waypoints, goal_heading)
        if args.hold:
            spin(node)
        else:
            # The publish is asynchronous; give it a moment to go out.
            node.wait(lambda: False, 0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()


if __name__ == "__main__":
    main()
