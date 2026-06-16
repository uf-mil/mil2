#!/usr/bin/env python3
"""
record_fwd: apply a constant force and record thruster efforts
and odometry for a fixed duration.

Publishes directly to /cmd_wrench and temporarily disables the PID controller
to observe raw drag forces without control smoothing.

Usage: record_fwd <forward_force> <vertical_force> <horizontal_force> <duration_in_seconds>
  forward_force         -- forward force in Newtons
  vertical_force        -- vertical force in Newtons
  horizontal_force      -- horizontal force in Newtons
  duration_in_seconds   -- seconds to apply force and record
"""

import csv
import datetime as dt
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from std_srvs.srv import SetBool
from subjugator_msgs.msg import ThrusterEfforts

LOG_RATE_HZ = 10.0

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

args = sys.argv[1:]
if len(args) != 4:
    print(
        "Usage:\n\trecord_fwd <forward_force> <vertical_force> <horizontal_force> <duration_in_seconds>",
    )
    exit(1)

try:
    forward_n = float(args[0])
    vertical_n = float(args[1])
    horizontal_n = float(args[2])
    duration_s = float(args[3])
except ValueError:
    print("All forces and <duration_in_seconds> must be numbers.")
    exit(1)

if duration_s <= 0:
    print("<duration_in_seconds> must be positive.")
    exit(1)

stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
logs_dir = Path.home() / "record-fwd-movement" / "logs"
logs_dir.mkdir(parents=True, exist_ok=True)
forces_str = f"fx{forward_n:.2f}_fz{vertical_n:.2f}_fy{horizontal_n:.2f}"
out_path = logs_dir / f"record_fwd_{forces_str}_{stamp}.csv"
counter = 1
while out_path.exists():
    out_path = logs_dir / f"record_fwd_{forces_str}_{stamp}_{counter}.csv"
    counter += 1

rclpy.init()
node = rclpy.create_node("forward_recorder")

# Disable the PID controller for the duration of the test
pid_client = node.create_client(SetBool, "/pid_controller/enable")
pid_status_at_start = pid_client.service_is_ready()
if pid_status_at_start:
    req = SetBool.Request()
    req.data = False
    future = pid_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print("Disabled pid_controller.")

current_pose = None
current_twist = None
current_thrust = None


def odom_cb(msg):
    global current_pose, current_twist
    current_pose = msg.pose.pose
    current_twist = msg.twist.twist


def thrust_cb(msg):
    global current_thrust
    current_thrust = msg


node.create_subscription(Odometry, "/odometry/filtered", odom_cb, 10)
node.create_subscription(ThrusterEfforts, "/thruster_efforts", thrust_cb, 10)
wrench_pub = node.create_publisher(Wrench, "/cmd_wrench", 10)

forward_wrench = Wrench()
forward_wrench.force.x = forward_n  # forward/backward
forward_wrench.force.y = horizontal_n  # left/right strafe
forward_wrench.force.z = vertical_n  # up/down depth

zero_wrench = Wrench()

csv_file = out_path.open("w", newline="")
csv_writer = csv.writer(csv_file)
header = [
    "t_sec",
    "input_fx",
    "input_fy",
    "input_fz",
    *THRUSTER_FIELDS,
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "roll_rate_deg_s",
    "pitch_rate_deg_s",
    "yaw_rate_deg_s",
]
csv_writer.writerow(header)

print(f"Logging to {out_path}")

start = time.monotonic()
period_s = 1.0 / LOG_RATE_HZ
next_log = start

try:
    while rclpy.ok() and (time.monotonic() - start) < duration_s:
        rclpy.spin_once(node, timeout_sec=0.001)
        now = time.monotonic()

        wrench_pub.publish(forward_wrench)

        if now >= next_log:
            t_sec = now - start

            thrust_vals = []
            for field in THRUSTER_FIELDS:
                if current_thrust is None:
                    thrust_vals.append(float("nan"))
                else:
                    thrust_vals.append(float(getattr(current_thrust, field)))

            if current_pose is None:
                x = y = z = float("nan")
                roll_deg = pitch_deg = yaw_deg = float("nan")
            else:
                x = float(current_pose.position.x)
                y = float(current_pose.position.y)
                z = float(current_pose.position.z)
                q = current_pose.orientation
                rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
                roll_deg, pitch_deg, yaw_deg = rot.as_euler("xyz", degrees=True)

            if current_twist is None:
                vx = vy = vz = float("nan")
                roll_rate_deg_s = pitch_rate_deg_s = yaw_rate_deg_s = float("nan")
            else:
                vx = float(current_twist.linear.x)
                vy = float(current_twist.linear.y)
                vz = float(current_twist.linear.z)
                roll_rate_deg_s = math.degrees(float(current_twist.angular.x))
                pitch_rate_deg_s = math.degrees(float(current_twist.angular.y))
                yaw_rate_deg_s = math.degrees(float(current_twist.angular.z))

            row = [f"{t_sec:.3f}"]

            row.append(f"{forward_n:.3f}")
            row.append(f"{horizontal_n:.3f}")
            row.append(f"{vertical_n:.3f}")

            for val in thrust_vals:
                row.append(f"{val:.3f}")

            row.append(f"{x:.3f}")
            row.append(f"{y:.3f}")
            row.append(f"{z:.3f}")

            row.append(f"{vx:.3f}")
            row.append(f"{vy:.3f}")
            row.append(f"{vz:.3f}")

            row.append(f"{roll_deg:.3f}")
            row.append(f"{pitch_deg:.3f}")
            row.append(f"{yaw_deg:.3f}")

            row.append(f"{roll_rate_deg_s:.3f}")
            row.append(f"{pitch_rate_deg_s:.3f}")
            row.append(f"{yaw_rate_deg_s:.3f}")

            csv_writer.writerow(row)
            csv_file.flush()

            next_log += period_s
except KeyboardInterrupt:
    print("Keyboard interrupt -- closing log.")
finally:
    wrench_pub.publish(zero_wrench)
    if pid_status_at_start:
        req = SetBool.Request()
        req.data = True
        future = pid_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        print("Re-enabled pid_controller.")
    csv_file.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"Done. Log written to {out_path}")
