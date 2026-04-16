import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

args = sys.argv[1:]
if len(args) == 0 or len(args) > 6:
    print("Usage: move_rel <dx> [dy] [dz] [droll] [dpitch] [dyaw]")
    exit(1)

args += [0] * (6 - len(args))
dx, dy, dz, droll_deg, dpitch_deg, dyaw_deg = map(float, args)

rclpy.init()
node = rclpy.create_node("move_relative_publisher")

current_pose = None


def odom_cb(msg):
    global current_pose
    current_pose = msg.pose.pose


node.create_subscription(Odometry, "/odometry/filtered", odom_cb, 10)

while rclpy.ok() and current_pose is None:
    rclpy.spin_once(node, timeout_sec=0.1)

body_rot = R.from_quat(
    [
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w,
    ],
)

abs_pos = np.array(
    [current_pose.position.x, current_pose.position.y, current_pose.position.z],
) + body_rot.apply([dx, dy, dz])

abs_rot = body_rot * R.from_euler("xyz", np.deg2rad([droll_deg, dpitch_deg, dyaw_deg]))
qx, qy, qz, qw = abs_rot.as_quat()

goal = Pose()
goal.position.x, goal.position.y, goal.position.z = abs_pos
goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w = (
    qx,
    qy,
    qz,
    qw,
)

pub = node.create_publisher(Pose, "/goal_pose", 10)
time.sleep(0.2)  # allow DDS match-up
pub.publish(goal)
node.get_logger().info(f"Published relative goal:\n{goal}")
time.sleep(0.3)
node.destroy_node()
rclpy.shutdown()
