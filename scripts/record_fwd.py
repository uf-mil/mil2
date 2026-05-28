#!/usr/bin/env python3
"""
record_fwd: record thruster efforts and orientation while the sub moves forward.

Usage: record_fwd X Y
  X -- meters to move forward (passed to move_rel)
  Y -- seconds to keep recording

Writes a timestamped CSV (extension .csv.log to match the repo's *.log
gitignore rule) to ~/sub_logs by default, or to $RECORD_FWD_LOG_DIR if set.
Each row holds a sample at 10 Hz containing the eight thruster efforts and
the robot's roll, pitch, and yaw in degrees.
"""

import csv
import datetime as dt
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from subjugator_msgs.msg import ThrusterEfforts

LOG_RATE_HZ = 10.0
MOVE_REL_DELAY_S = 0.3  # let DDS finish matching subscriptions before firing

THRUSTER_FIELDS = [
    "thrust_flh",
    "thrust_frh",
    "thrust_blh",
    "thrust_brh",
    "thrust_flv",
    "thrust_frv",
    "thrust_blv",
    "thrust_brv",
]

# move_rel.py is a sibling of this file in mil2/scripts/
MOVE_REL_PATH = Path(__file__).resolve().parent / "move_rel.py"


args = sys.argv[1:]
if len(args) != 2:
    print("Usage: record_fwd X Y")
    print("  X -- meters to move forward")
    print("  Y -- seconds to record")
    exit(1)

try:
    distance_m = float(args[0])
    duration_s = float(args[1])
except ValueError:
    print("X and Y must be numbers.")
    exit(1)

if duration_s <= 0:
    print("Y must be positive.")
    exit(1)

# Pick a non-colliding output path. Default to ~/sub_logs; override with
# RECORD_FWD_LOG_DIR. Extension .csv.log matches the repo's *.log gitignore
# rule, so files won't get committed if the dir ever lands inside the repo.
default_log_dir = Path.home() / "sub_logs"
log_dir = Path(os.environ.get("RECORD_FWD_LOG_DIR", default_log_dir))
log_dir.mkdir(parents=True, exist_ok=True)

stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
out_path = log_dir / f"record_fwd_{stamp}.csv.log"
counter = 1
while out_path.exists():
    out_path = log_dir / f"record_fwd_{stamp}_{counter}.csv.log"
    counter += 1

rclpy.init()
node = rclpy.create_node("forward_recorder")

current_pose = None
current_thrust = None


def odom_cb(msg):
    global current_pose
    current_pose = msg.pose.pose


def thrust_cb(msg):
    global current_thrust
    current_thrust = msg


node.create_subscription(Odometry, "/odometry/filtered", odom_cb, 10)
node.create_subscription(ThrusterEfforts, "/thruster_efforts", thrust_cb, 10)

csv_file = out_path.open("w", newline="")
csv_writer = csv.writer(csv_file)
header = ["t_sec", *THRUSTER_FIELDS, "roll_deg", "pitch_deg", "yaw_deg"]
csv_writer.writerow(header)

print(f"Logging to {out_path}")
print(f"Moving forward {distance_m} m, recording for {duration_s} s...")

start = time.monotonic()
period_s = 1.0 / LOG_RATE_HZ
next_log = start
move_rel_sent = False

try:
    while rclpy.ok() and (time.monotonic() - start) < duration_s:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.monotonic()

        if not move_rel_sent and (now - start) >= MOVE_REL_DELAY_S:
            print(f"Sending move_rel {distance_m} at t={now - start:.2f}s\n")
            try:
                subprocess.Popen(["python3", str(MOVE_REL_PATH), str(distance_m)])
            except OSError as exc:
                print(f"Failed to launch move_rel: {exc}")
            move_rel_sent = True

        if now >= next_log:
            t_sec = now - start

            # Pull thruster snapshot
            thrust_vals = []
            for field in THRUSTER_FIELDS:
                if current_thrust is None:
                    thrust_vals.append(float("nan"))
                else:
                    thrust_vals.append(float(getattr(current_thrust, field)))

            # Pull orientation snapshot
            if current_pose is None:
                roll_deg = float("nan")
                pitch_deg = float("nan")
                yaw_deg = float("nan")
            else:
                q = current_pose.orientation
                rot = R.from_quat([q.x, q.y, q.z, q.w])
                roll_deg, pitch_deg, yaw_deg = rot.as_euler("xyz", degrees=True)

            row = [f"{t_sec:.4f}"]
            for val in thrust_vals:
                row.append(f"{val:.6f}")
            row.append(f"{roll_deg:.4f}")
            row.append(f"{pitch_deg:.4f}")
            row.append(f"{yaw_deg:.4f}")
            csv_writer.writerow(row)
            csv_file.flush()

            next_log += period_s
except KeyboardInterrupt:
    print("\nKeyboard interrupt -- closing log.")
finally:
    csv_file.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"\nDone. Log written to {out_path}")
