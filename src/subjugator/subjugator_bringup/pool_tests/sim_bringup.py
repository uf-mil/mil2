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
  # end the settle when the sub is measurably still instead of after N wall
  # seconds (--settle is WALL time, and RTF on this box varies ~20x run to run,
  # so it buys wildly different amounts of real settling):
  python3 sim_bringup.py --x -7.702 --y 13.972 --settle-until-still
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

# Wall-clock deadlock guard for the convergence gate. The gate is capped in SIM
# seconds, but a sim whose /clock is not ticking at all (gz dead, or paused
# again behind our back) can never reach that cap, so the loop would spin
# forever. This is NOT a wall cap on the gate: it only fires after the sim clock
# has made no progress whatsoever for this long.
SETTLE_STALL_WALL_S = 60.0


def settled(twist_mags, tol, need):
    """True once the last `need` twist magnitudes are all below `tol`.

    Wall-clock settling does not survive this box: RTF varies ~20x run to run, so
    a fixed --settle buys wildly different amounts of actual settling. Waiting on
    the vehicle actually being still is RTF-independent.
    """
    if len(twist_mags) < need:
        return False
    return all(m < tol for m in twist_mags[-need:])


def positive_int(v):
    """argparse type for a count that must be at least 1.

    `--stable-samples 0` would make `settled()` ask whether all of an empty
    window is small, which is vacuously true -- a run with no odometry at all
    would report `converged` immediately. Reject it instead of clamping, so the
    mistake is visible.
    """
    i = int(v)
    if i < 1:
        raise argparse.ArgumentTypeError(
            f"must be at least 1 (got {i}; 0 would report convergence with no data)",
        )
    return i


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

    def twist_magnitude(self):
        """Largest linear/angular velocity component in the latest odometry."""
        if self.odom is None:
            return None
        t = self.odom.twist.twist
        return max(
            abs(t.linear.x),
            abs(t.linear.y),
            abs(t.linear.z),
            abs(t.angular.x),
            abs(t.angular.y),
            abs(t.angular.z),
        )

    def odom_stamp(self):
        """Stamp of the latest odometry message, or None if none has arrived.

        The settle gate counts DISTINCT messages, not polls: `spin_for` waits in
        wall time while odometry is published on sim time, so at RTF 0.05 a
        0.5 s poll advances the sim by less than one odometry period and the
        same message is read over and over. Counting those as separate samples
        would let the gate "converge" on ten copies of a single instant -- e.g.
        the moment a damped oscillation crosses zero velocity -- which is the
        RTF-dependence this whole gate exists to remove.
        """
        if self.odom is None:
            return None
        s = self.odom.header.stamp
        return (s.sec, s.nanosec)

    def sim_now(self):
        """Seconds on the /clock (the node runs with use_sim_time)."""
        return self.get_clock().now().nanoseconds / 1e9

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
        help="fixed WALL seconds to settle under control (default: 25)",
    )
    ap.add_argument(
        "--settle-until-still",
        action="store_true",
        help="hold until the sub is measurably still instead of a fixed wall wait",
    )
    ap.add_argument(
        "--twist-tol",
        type=float,
        default=0.02,
        help="max velocity component counted as still (m/s, rad/s)",
    )
    ap.add_argument(
        "--stable-samples",
        type=positive_int,
        default=10,
        help="consecutive still odometry MESSAGES required (not poll count)",
    )
    ap.add_argument(
        "--settle-cap-sim",
        type=float,
        default=40.0,
        help="sim-second cap on the convergence gate",
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

    if args.settle_until_still:
        print(
            f"[5] hold until still (tol {args.twist_tol}, "
            f"{args.stable_samples} messages, cap {args.settle_cap_sim:.0f} sim-s) ...",
        )
        mags = []
        last_stamp = None
        start_sim = n.sim_now()
        last_sim = start_sim
        last_progress_wall = time.time()
        converged = False
        stalled = False
        while True:
            n.goal(hx, hy, hz)
            n.spin_for(0.5)
            stamp = n.odom_stamp()
            mag = n.twist_magnitude()
            # One sample per DISTINCT odometry message (see Bringup.odom_stamp).
            # If odometry stops arriving the list stops growing and the gate
            # rides to the sim-second cap, which is the honest answer.
            if mag is not None and stamp != last_stamp:
                last_stamp = stamp
                mags.append(mag)
            now_sim = n.sim_now()
            elapsed = now_sim - start_sim
            if settled(mags, args.twist_tol, args.stable_samples):
                converged = True
                break
            if elapsed >= args.settle_cap_sim:
                break
            if now_sim != last_sim:
                last_sim = now_sim
                last_progress_wall = time.time()
            elif time.time() - last_progress_wall >= SETTLE_STALL_WALL_S:
                stalled = True
                break
        elapsed = max(0.0, n.sim_now() - start_sim)
        if stalled:
            print("!! sim clock is not advancing; settle gate gave up (is gz alive?)")
        # Machine-readable: run_task parses this line into the run report.
        state = "converged" if converged else "capped"
        print(f"settle: {state} after {elapsed:.1f} sim-s")
    else:
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
                    else "    WARN: offset still large; sub may not have settled -- raise --settle / --settle-cap-sim."
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
