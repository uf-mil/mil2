#!/usr/bin/env python3
"""
Task 1, Safe Passage, at the Core tier.

  in   /beacons                   mil_msgs/PerceptionObjectArray, beacon_node's buoys
       odometry                   nav_msgs/Odometry, in the buoys' frame
       /prop/run_start            std_msgs/UInt32, from the OCS at RunStart
       gps/fix, imu/data          for the report's latitudes and longitudes
  out  /goal_pose                 geometry_msgs/PoseStamped, for the planner
       /mission/current_task      std_msgs/String, latched, for the heartbeat
       /prop/safe_passage_report  prop_ocs_interfaces/SafePassageReport

Start it before the run. It does nothing until the OCS passes on RunStart, then
it will

  1. say the task is SAFE_PASSAGE, which is what turns the lights on, and drive
     to field_latlon if it was given
  2. once the lights have had light_delay to come on, have beacon_node forget
     what it saw while they were off
  3. visit each buoy within survey_radius whose light is not known, nearest
     first, until none are left; once the entry and exit are known, only
     those within passage_margin of the way between them
  4. circle the FLASHING_BLUE entry clockwise
  5. cross to the STEADY_BLUE exit, passing beside each red buoy on its port
     side and each green one on its starboard side, so red goes by to
     starboard and green to port; the planner keeps clear of everything else
  6. circle the exit counter-clockwise, and say the task is NONE

The OCS gets a report whenever what the boat knows changes. Goals are sent in
the buoys' frame, which has to be the planner's.
"""

import math

import numpy as np
import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from mil_msgs.msg import PerceptionObjectArray
from nav_msgs.msg import Odometry
from prop_ocs_interfaces.msg import Buoy, SafePassageReport
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt32
from std_srvs.srv import SetBool, Trigger

from prop_missions.geo import GeoReference, yaw_of

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ENTRY, EXIT = "FLASHING_BLUE", "STEADY_BLUE"
SIDE = {"FLASHING_RED": 1.0, "FLASHING_GREEN": -1.0}  # +1: pass on its port side
KNOWN = {"OFF", "FLASHING_RED", "FLASHING_GREEN", ENTRY, EXIT}


def circle(center, start, radius, clockwise, step=math.radians(45)):
    """
    Waypoints (x, y, heading) once around center and two steps more, starting
    on the circle nearest start, as reaching each waypoint early cuts corners.
    Clockwise as seen from above.
    """
    cx, cy = center
    angle = math.atan2(start[1] - cy, start[0] - cx)
    turn = -1.0 if clockwise else 1.0
    steps = int(round(2 * math.pi / step)) + 2
    return [
        (
            cx + radius * math.cos(angle + turn * i * step),
            cy + radius * math.sin(angle + turn * i * step),
            angle + turn * (i * step + math.pi / 2),
        )
        for i in range(steps + 1)
    ]


def off_segment(point, a, b):
    """How far point is from the segment a-b."""
    p, a, b = (np.asarray(v, float) for v in (point, a, b))
    t = np.clip((p - a) @ (b - a) / max((b - a) @ (b - a), 1e-9), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


def passage(entry, exit_, buoys, offset=2.0, abreast=2.5):
    """
    Waypoints (x, y, distance along) from entry to exit: beside each red buoy
    on its port side, each green one on its starboard side, and between a red
    and a green that lie abreast with the red to starboard. buoys are
    (x, y, state); the rest are only obstacles.
    """
    entry, exit_ = np.asarray(entry, float), np.asarray(exit_, float)
    length = np.linalg.norm(exit_ - entry)
    along = (exit_ - entry) / length
    left = np.array([-along[1], along[0]])
    gates = []
    for x, y, state in buoys:
        p = np.array([x, y], float)
        s = float((p - entry) @ along)
        if state in SIDE and 0 < s < length:
            gates.append((s, p, SIDE[state]))
    gates.sort(key=lambda g: g[0])

    points = []
    i = 0
    while i < len(gates):
        s, p, side = gates[i]
        if (
            i + 1 < len(gates)
            and gates[i + 1][0] - s < abreast
            and gates[i + 1][2] != side
        ):
            red, green = (p, gates[i + 1][1]) if side > 0 else (gates[i + 1][1], p)
            if (red - green) @ left < 0:
                points.append((red + green) / 2)
                i += 2
                continue
        points.append(p + side * offset * left)
        i += 1
    return [(float(q[0]), float(q[1]), float((q - entry) @ along)) for q in points]


class SafePassage(Node):
    def __init__(self):
        super().__init__("safe_passage")
        p = self.declare_parameter
        self.run_id = None
        self.field_latlon = list(p("field_latlon", [0.0, 0.0]).value)
        self.light_delay = p("light_delay", 5.0).value
        self.survey_radius = p("survey_radius", 60.0).value
        self.passage_margin = p("passage_margin", 10.0).value
        self.survey_time = p("survey_time", 300.0).value
        self.look_distance = p("look_distance", 5.0).value
        self.look_time = p("look_time", 8.0).value
        self.circle_radius = p("circle_radius", 3.0).value
        self.gate_offset = p("gate_offset", 2.0).value
        self.clearance = p("clearance", 1.2).value
        self.reach = p("reach", 1.5).value
        self.goal_timeout = p("goal_timeout", 60.0).value
        self.max_time = p("max_time", 900.0).value

        self.geo = GeoReference(self)
        self.buoys = {}  # id -> (x, y, state, confidence)
        self.frame = None
        self.pose = None
        self.started = None
        self.center = None  # where the survey looks around
        self.phase = "WAIT"
        self.since = None
        self.route = []  # this leg's waypoints left: (x, y, heading)
        self.goal_sent = None
        self.target = None  # the buoy being looked at
        self.arrived = None
        self.visited = set()
        self.entry = self.exit = None
        self.passed = 0.0  # how far along the passage the last gate was
        self.reported = (None, -math.inf)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.task_pub = self.create_publisher(String, "/mission/current_task", LATCHED)
        self.report_pub = self.create_publisher(
            SafePassageReport,
            "/prop/safe_passage_report",
            10,
        )
        self.planner = self.create_client(SetBool, "/planner/enable")
        self.reset_beacons = self.create_client(Trigger, "/beacons/reset")
        self.create_subscription(PerceptionObjectArray, "/beacons", self.on_beacons, 10)
        self.create_subscription(Odometry, "odometry", self.on_odometry, 10)
        self.create_subscription(UInt32, "/prop/run_start", self.on_run_start, 10)
        self.create_timer(0.2, self.step)
        self.get_logger().info("waiting for RunStart")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_run_start(self, msg):
        if self.phase == "WAIT":
            self.get_logger().info(f"RunStart for run {msg.data}")
            self.run_id = msg.data

    def on_odometry(self, msg):
        self.geo.on_odometry(msg)
        if self.frame is not None and msg.header.frame_id != self.frame:
            self.get_logger().error(
                f"odometry is in {msg.header.frame_id} but the buoys are in "
                f"{self.frame}; remap odometry",
                throttle_duration_sec=5.0,
            )
            return
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def on_beacons(self, msg):
        if msg.objects:
            self.frame = msg.objects[0].header.frame_id
        self.buoys = {
            o.id: (
                o.pose.position.x,
                o.pose.position.y,
                o.labeled_classification,
                o.classification_confidence,
            )
            for o in msg.objects
        }
        if self.phase not in ("WAIT", "DONE"):
            self.report()

    def step(self):
        if self.pose is None or self.frame is None:
            return
        running = self.started is not None and self.phase != "DONE"
        if running and self.now() - self.started > self.max_time:
            self.finish("out of time")
            return
        getattr(self, f"step_{self.phase.lower()}")()

    def enter(self, phase, route=()):
        self.get_logger().info(f"{self.phase} -> {phase}")
        self.phase = phase
        self.since = self.now()
        self.go(route)

    # ----- the phases -----

    def step_wait(self):
        if self.run_id is None:
            return
        self.started = self.now()
        self.task_pub.publish(String(data="SAFE_PASSAGE"))
        self.call(self.planner, SetBool.Request(data=True))
        self.center = self.pose[:2]
        if any(self.field_latlon):
            field = self.geo.local(*self.field_latlon)
            if field is None:
                self.finish("no GPS fix to find field_latlon by")
                return
            self.center = field
            self.enter(
                "APPROACH",
                [
                    (
                        *field,
                        math.atan2(field[1] - self.pose[1], field[0] - self.pose[0]),
                    ),
                ],
            )
            return
        self.enter("LIGHTS")

    def step_approach(self):
        self.follow()
        if not self.route:
            self.enter("LIGHTS")

    def step_lights(self):
        if self.now() - self.started >= self.light_delay:
            self.call(self.reset_beacons, Trigger.Request())
            self.enter("SURVEY")

    def step_survey(self):
        if self.route:
            self.follow()
            if not self.route:
                self.arrived = self.now()
            return
        if self.target is not None:
            state = self.buoys.get(self.target, (0, 0, "UNKNOWN"))[2]
            if state == "UNKNOWN" and self.now() - self.arrived < self.look_time:
                return
            self.visited.add(self.target)
            self.target = None
        unknown = [
            (math.dist(b[:2], self.pose[:2]), i)
            for i, b in self.buoys.items()
            if b[2] == "UNKNOWN"
            and i not in self.visited
            and math.dist(b[:2], self.center) <= self.survey_radius
        ]
        # Once both ends are known, only buoys near the way between them matter.
        ends = (self.best(ENTRY), self.best(EXIT))
        if None not in ends:
            unknown = [
                u
                for u in unknown
                if off_segment(self.buoys[u[1]][:2], *ends) <= self.passage_margin
            ]
        if not unknown or self.now() - self.since > self.survey_time:
            self.choose_ends()
            return
        self.target = min(unknown)[1]
        x, y = self.buoys[self.target][:2]
        away = math.atan2(self.pose[1] - y, self.pose[0] - x)
        self.get_logger().info(f"looking at #{self.target}")
        self.go(
            [
                (
                    x + self.look_distance * math.cos(away),
                    y + self.look_distance * math.sin(away),
                    away + math.pi,
                ),
            ],
        )

    def choose_ends(self):
        self.entry, self.exit = self.best(ENTRY), self.best(EXIT)
        if self.entry is None or self.exit is None:
            missing = " and ".join(
                name
                for name, at in (("entry", self.entry), ("exit", self.exit))
                if at is None
            )
            self.finish(f"no {missing} found")
            return
        self.enter(
            "ENTRY",
            circle(self.entry, self.pose, self.circle_radius, clockwise=True),
        )

    def step_entry(self):
        self.follow()
        if not self.route:
            self.passed = self.along(self.pose)
            self.enter("TRANSIT")

    def step_transit(self):
        if self.route:
            self.follow()
            return
        heading = math.atan2(self.exit[1] - self.entry[1], self.exit[0] - self.entry[0])
        floor = max(self.passed, self.along(self.pose)) + 0.5
        ahead = [
            g
            for g in passage(self.entry, self.exit, self.known(), self.gate_offset)
            if g[2] > floor
        ]
        if ahead:
            x, y, self.passed = ahead[0]
            self.go([(x, y, heading)])
        else:
            self.enter(
                "EXIT",
                circle(self.exit, self.pose, self.circle_radius, clockwise=False),
            )

    def step_exit(self):
        self.follow()
        if not self.route:
            self.finish("done")

    def step_done(self):
        pass

    def finish(self, why):
        self.get_logger().info(f"Safe Passage over: {why}")
        self.report(force=True)
        self.task_pub.publish(String(data="NONE"))
        self.enter("DONE")

    # ----- getting about -----

    def go(self, route):
        """Follow route next, leaving out waypoints too close to a buoy."""
        self.route = [
            w
            for w in route
            if all(
                math.dist(w[:2], b[:2]) >= self.clearance for b in self.buoys.values()
            )
        ]
        self.goal_sent = None

    def follow(self):
        """Send the next waypoint, and drop it once reached or given up on."""
        if not self.route:
            return
        x, y, heading = self.route[0]
        if self.goal_sent is None:
            self.send_goal(x, y, heading)
        if math.dist((x, y), self.pose[:2]) <= self.reach:
            self.route.pop(0)
            self.goal_sent = None
        elif self.now() - self.goal_sent > self.goal_timeout:
            self.get_logger().warn(f"gave up on ({x:.1f}, {y:.1f})")
            self.route.pop(0)
            self.goal_sent = None

    def send_goal(self, x, y, heading):
        goal = PoseStamped()
        goal.header.frame_id = self.frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z = math.sin(heading / 2)
        goal.pose.orientation.w = math.cos(heading / 2)
        self.goal_pub.publish(goal)
        self.goal_sent = self.now()

    def along(self, point):
        """How far point is along the way from entry to exit."""
        ex, ey = self.exit[0] - self.entry[0], self.exit[1] - self.entry[1]
        return (
            (point[0] - self.entry[0]) * ex + (point[1] - self.entry[1]) * ey
        ) / math.hypot(ex, ey)

    def call(self, client, request):
        if not client.service_is_ready():
            self.get_logger().warn(
                f"{client.srv_name} is not up; carrying on without it",
            )
        client.call_async(request)

    # ----- what the boat knows -----

    def known(self):
        """(x, y, state) of the buoys in the survey area whose light is known."""
        return [
            b[:3]
            for b in self.buoys.values()
            if b[2] in KNOWN and math.dist(b[:2], self.center) <= self.survey_radius
        ]

    def best(self, state):
        """The most certain buoy in the survey area showing state, as (x, y)."""
        candidates = [
            b
            for b in self.buoys.values()
            if b[2] == state and math.dist(b[:2], self.center) <= self.survey_radius
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda b: b[3])[:2]

    def report(self, force=False):
        """
        Tell the OCS what the boat knows: when it changes, at most once a
        second, and again every 5 s.
        """
        if self.center is None:
            return
        if not self.geo.ready():
            self.get_logger().warn(
                "no GPS fix yet, holding the report",
                throttle_duration_sec=10.0,
            )
            return
        buoys = sorted(self.known())
        entry, exit_ = self.best(ENTRY), self.best(EXIT)
        content = [(round(x), round(y), s) for x, y, s in buoys]
        since = self.now() - self.reported[1]
        if not force and (since < 1.0 or (content == self.reported[0] and since < 5.0)):
            return
        msg = SafePassageReport()
        msg.entry_position = self.geopoint(entry)
        msg.exit_position = self.geopoint(exit_)
        for x, y, state in buoys:
            msg.buoys.append(Buoy(position=self.geopoint((x, y)), state=state))
        self.report_pub.publish(msg)
        self.reported = (content, self.now())

    def geopoint(self, point):
        if point is None:
            return GeoPoint(latitude=math.nan, longitude=math.nan)
        latitude, longitude = self.geo.latlon(*point)
        return GeoPoint(latitude=latitude, longitude=longitude)


def main():
    rclpy.init()
    node = SafePassage()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
