#!/usr/bin/env python3
"""Correct-order sim bring-up: settle under control BEFORE anchoring the EKF.

Why this exists
---------------
The localization EKF fuses no absolute x/y (IMU = orientation+angular+accel,
DVL = velocity, depth = z only), so horizontal position is pure dead reckoning:
whatever pose you `set_pose` it to is trusted forever, because no sensor can ever
contradict it. The obvious bring-up -- teleport the sub, then immediately reset
the EKF to that pose -- anchors the estimate to a sub that is still drifting
(the controller is off, so it floats/settles for a few seconds). That freezes a
~0.25 m offset between odom and the true pose for the entire run. It is NOT a
filter fault; the EKF tracks motion correctly, it was just handed a wrong origin.

The offset cancels inside LockTargetXY (its gate differences two odom-frame
points), so it does not break the grasp -- but it corrupts every odom-vs-world
measurement and makes the controller park the physical sub off the world goal.

The fix is ordering, proven in sim: hold the sub dead-still under the controller
first, and only THEN anchor the EKF -- onto the settled pose. Offset drops under
1 cm and stays there. This script encodes that order so no run has to remember it.

Sequence
--------
  1. unpause gz (the sim launches paused, headless has no GUI play button)
  2. enable the EKF
  3. optionally teleport the sub to a target pose (else use where it is now)
  4. publish a hold goal = the target pose, THEN enable the controller
     (goal-first avoids the zero-setpoint fling)
  5. hold, letting the sub go dead still
  6. read the settled truth pose from gz and set_pose the EKF onto it
  7. leave EKF + controller enabled, controller latched on the hold goal;
     the mission you run next takes over /goal_pose

Sim only: step 6 reads gz ground truth, which does not exist on hardware. On the
real sub the vehicle sits still on its stand at enable time, so the drift this
guards against does not occur.

Usage
-----
  # settle + anchor wherever the sub currently is (GUI-positioned):
  python3 sim_bringup.py
  # teleport to the over-table hover first, then settle + anchor:
  python3 sim_bringup.py --x -7.702 --y 13.972 --z -0.35 --settle 25
"""

import argparse
import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import SetPose
from std_srvs.srv import Empty, SetBool

WORLD = "robosub_2025"


def gz_unpause():
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{WORLD}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            "pause: false",
        ],
        capture_output=True,
        timeout=15,
    )


def gz_teleport(x, y, z):
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{WORLD}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'name: "sub9", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}',
        ],
        capture_output=True,
        timeout=15,
    )


def gz_truth_xyz():
    out = subprocess.run(
        ["gz", "topic", "-e", "-t", f"/world/{WORLD}/dynamic_pose/info", "-n", "1"],
        capture_output=True,
        text=True,
        timeout=15,
    ).stdout
    for b in re.findall(r"pose\s*\{(.*?)\n\}", out, re.S):
        nm = re.search(r'name:\s*"([^"]+)"', b)
        if nm and nm.group(1) == "sub9":
            p = re.search(r"position\s*\{([^}]*)\}", b)
            d = dict(re.findall(r"(\w+):\s*(-?[\d.eE+-]+)", p.group(1)))
            return tuple(float(d.get(k, 0)) for k in "xyz")
    return None


class Bringup(Node):
    def __init__(self):
        super().__init__("sim_bringup")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.odom = None
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            lambda m: setattr(self, "odom", m),
            20,
        )
        self.goal_pub = self.create_publisher(Pose, "/goal_pose", 10)

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, T, name, req, t=10.0):
        c = self.create_client(T, name)
        if not c.wait_for_service(timeout_sec=t):
            self.get_logger().error(f"service {name} unavailable")
            return None
        f = c.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=t)
        return f.result()

    def set_pose(self, x, y, z):
        req = SetPose.Request()
        req.pose.header.frame_id = "odom"
        req.pose.pose.pose.position.x = x
        req.pose.pose.pose.position.y = y
        req.pose.pose.pose.position.z = z
        req.pose.pose.pose.orientation.w = 1.0
        self.call(SetPose, "/subjugator_localization/set_pose", req)

    def goal(self, x, y, z):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        self.goal_pub.publish(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, help="teleport target x (omit = stay put)")
    ap.add_argument("--y", type=float, help="teleport target y")
    ap.add_argument("--z", type=float, default=-0.35, help="teleport/hover z")
    ap.add_argument(
        "--settle",
        type=float,
        default=25.0,
        help="settle seconds under control",
    )
    args = ap.parse_args()

    rclpy.init()
    n = Bringup()

    print("[1] unpause gz")
    gz_unpause()
    n.spin_for(2)

    print("[2] enable EKF")
    n.call(Empty, "/subjugator_localization/enable", Empty.Request())
    n.spin_for(2)

    if args.x is not None and args.y is not None:
        print(f"[3] teleport sub -> ({args.x}, {args.y}, {args.z})")
        gz_teleport(args.x, args.y, args.z)
        n.spin_for(2)
        hx, hy, hz = args.x, args.y, args.z
    else:
        t = gz_truth_xyz()
        if not t:
            print("!! no gz truth; is the sim up?")
            return 1
        hx, hy, hz = t[0], t[1], args.z
        print(
            f"[3] no teleport; holding at current pose ({hx:.3f}, {hy:.3f}, {hz:.3f})",
        )

    # provisional anchor so the controller has a frame to hold against
    n.set_pose(hx, hy, hz)
    n.spin_for(1)

    print("[4] publish hold goal FIRST, then enable controller (goal-first: no fling)")
    n.goal(hx, hy, hz)
    n.spin_for(1)
    n.call(SetBool, "/pid_controller/enable", SetBool.Request(data=True))

    print(f"[5] hold {args.settle:.0f}s so the sub goes dead still ...")
    steps = int(args.settle)
    for _ in range(steps):
        n.goal(hx, hy, hz)
        n.spin_for(1.0)

    print("[6] re-anchor EKF onto the SETTLED truth pose (the whole point)")
    t = gz_truth_xyz()
    if t:
        n.set_pose(t[0], t[1], hz)
        n.spin_for(0.5)
        if n.odom:
            p = n.odom.pose.pose.position
            off = math.hypot(p.x - t[0], p.y - t[1])
            print(
                f"    settled truth=({t[0]:.3f},{t[1]:.3f})  odom=({p.x:.3f},{p.y:.3f})  offset={off:.4f} m",
            )
            print(
                (
                    "    OK: offset < 0.02 m means the anchor is clean."
                    if off < 0.02
                    else "    WARN: offset still large; sub may not have settled -- raise --settle."
                ),
            )

    print(
        "[7] EKF + controller enabled, holding. Run your mission now; it takes over /goal_pose.",
    )
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
