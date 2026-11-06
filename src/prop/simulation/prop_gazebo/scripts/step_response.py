#!/usr/bin/env python3
"""
Step one axis of the controller's goal and report how the boat answers.

    ros2 run prop_gazebo step_response.py surge 3     # 3 m ahead
    ros2 run prop_gazebo step_response.py sway 2      # 2 m to port
    ros2 run prop_gazebo step_response.py yaw 90      # 90 degrees to port
    ros2 run prop_gazebo step_response.py surge 3 --duration 20

Publishes the step on goal/trajectory, which is pid_controller's input, so the
trajectory planner has to be idle. The step is taken from whichever estimate
the controller is subscribed to, sim/ground_truth or odometry/filtered/local.
Watches sim/ground_truth and prints rise time (10 to 90 %), overshoot,
settling time (within 5 %), the error at the end, how far the other two axes
strayed meanwhile, and how far the estimate drifted from the truth. The
controller has to be running: start-controller, or the sim's autostart.

Between steps,  ros2 param set /pid_controller kp "[...]"  changes gains live,
and reset-controller zeroes the integrators and makes the current pose the goal.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node

AXES = ("surge", "sway", "yaw")


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def body_axes(origin, msg: Odometry):
    """(surge, sway, yaw) of a pose relative to an origin (x, y, yaw)."""
    x0, y0, yaw0 = origin
    p = msg.pose.pose.position
    dx, dy = p.x - x0, p.y - y0
    return (
        math.cos(yaw0) * dx + math.sin(yaw0) * dy,
        -math.sin(yaw0) * dx + math.cos(yaw0) * dy,
        wrap(yaw_of(msg.pose.pose.orientation) - yaw0),
    )


class Stepper(Node):
    def __init__(self):
        super().__init__("step_response")
        self.odom = None
        self.truth = None
        self.samples = []  # (t, surge, sway, yaw) of the truth since the step began
        self.start = None  # truth (x, y, yaw, t) at the step
        self.odom_start = None  # estimate (x, y, yaw) at the step
        self.last_yaw = None  # truth yaw last sample, to unwrap a turn past 180
        self.create_subscription(Odometry, "sim/ground_truth", self.on_truth, 10)
        self.goal_pub = self.create_publisher(Pose, "goal/trajectory", 10)

    def follow_controller(self) -> str:
        """Subscribe to whichever estimate pid_controller closes the loop on."""
        topic = "/odometry/filtered/local"
        if any(
            s.node_name == "pid_controller"
            for s in self.get_subscriptions_info_by_topic("/sim/ground_truth")
        ):
            topic = "/sim/ground_truth"
        self.create_subscription(Odometry, topic, self.on_odom, 10)
        return topic

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def on_truth(self, msg: Odometry) -> None:
        self.truth = msg
        if self.start is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.start[3]
        surge, sway, yaw = body_axes(self.start[:3], msg)
        if self.samples:
            yaw = self.samples[-1][3] + wrap(
                yaw_of(msg.pose.pose.orientation) - self.last_yaw,
            )
        self.last_yaw = yaw_of(msg.pose.pose.orientation)
        self.samples.append((t, surge, sway, yaw))

    def drift(self):
        """How far the estimate's idea of the step differs from the truth's."""
        seen = body_axes(self.odom_start, self.odom)
        return tuple(s - t for s, t in zip(seen, self.samples[-1][1:]))

    def step(self, axis: str, size: float) -> None:
        """Goal = the estimate the controller sees, moved by the step in body axes."""
        pose = self.odom.pose.pose
        yaw = yaw_of(pose.orientation)
        goal = Pose()
        goal.position.x = pose.position.x
        goal.position.y = pose.position.y
        if axis == "surge":
            goal.position.x += size * math.cos(yaw)
            goal.position.y += size * math.sin(yaw)
        elif axis == "sway":
            goal.position.x += -size * math.sin(yaw)
            goal.position.y += size * math.cos(yaw)
        else:
            yaw += size
        goal.orientation.z = math.sin(0.5 * yaw)
        goal.orientation.w = math.cos(0.5 * yaw)

        truth = self.truth
        stamp = truth.header.stamp
        self.start = (
            truth.pose.pose.position.x,
            truth.pose.pose.position.y,
            yaw_of(truth.pose.pose.orientation),
            stamp.sec + stamp.nanosec * 1e-9,
        )
        self.odom_start = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.goal_pub.publish(goal)


def report(samples, axis: str, size: float, drift) -> None:
    index = AXES.index(axis) + 1
    response = [s[index] for s in samples]
    times = [s[0] for s in samples]
    unit = "deg" if axis == "yaw" else "m"
    scale = math.degrees(1.0) if axis == "yaw" else 1.0

    def first_time(fraction):
        return next((t for t, r in zip(times, response) if r / size >= fraction), None)

    t10, t90 = first_time(0.1), first_time(0.9)
    peak = max(response, key=lambda r: r / size)
    settled = next(
        (
            times[i]
            for i in range(len(times))
            if all(abs(r - size) <= 0.05 * abs(size) for r in response[i:])
        ),
        None,
    )
    print(f"{axis} step of {size * scale:.1f} {unit} over {times[-1]:.1f} s")
    print(
        (
            f"  rise 10-90%   {t90 - t10:6.2f} s"
            if t10 is not None and t90 is not None
            else "  rise 10-90%   never reached 90%"
        ),
    )
    print(f"  overshoot     {100.0 * (peak - size) / size:6.1f} %")
    print(
        (
            f"  settled (5%)  {settled:6.2f} s"
            if settled is not None
            else "  settled (5%)  not within the window"
        ),
    )
    print(f"  final error   {(size - response[-1]) * scale:6.2f} {unit}")
    for other in AXES:
        if other != axis:
            i = AXES.index(other) + 1
            stray = max(abs(s[i]) for s in samples)
            print(
                f"  {other:5} strayed {stray * (math.degrees(1.0) if other == 'yaw' else 1.0):6.2f} {'deg' if other == 'yaw' else 'm'}",
            )
    print(
        f"  estimate off  {drift[index - 1] * scale:6.2f} {unit} from the truth at the end",
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("axis", choices=AXES)
    parser.add_argument("size", type=float, help="metres, or degrees for yaw")
    parser.add_argument("--duration", type=float, default=15.0, help="seconds to watch")
    args = parser.parse_args()
    size = math.radians(args.size) if args.axis == "yaw" else args.size

    rclpy.init()
    node = Stepper()
    rclpy.spin_once(
        node,
        timeout_sec=1.0,
    )  # let the graph fill in before asking who listens to what
    estimate = node.follow_controller()
    deadline = time.monotonic() + 10.0
    while (node.odom is None or node.truth is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None or node.truth is None:
        sys.exit(
            f"no {estimate} or sim/ground_truth after 10 s; is the sim up and localization on?",
        )

    node.step(args.axis, size)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if len(node.samples) < 2:
        sys.exit("no ground truth arrived during the step")
    print(f"controller on {estimate}")
    report(node.samples, args.axis, size, node.drift())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
