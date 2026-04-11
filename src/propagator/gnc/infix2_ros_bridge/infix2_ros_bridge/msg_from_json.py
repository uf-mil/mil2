# a collection of funciton that take the json from the tcp stream and convert them into the corresponding ros2 message type

import json

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3, AccelStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

# chat also wrote this
# data is json not a string
def accel_from_json(data) -> AccelStamped:
    msg = AccelStamped()

    # --- Header ---
    msg.header = Header()
    msg.header.frame_id = data.get("frame_id", "")

    timestamp = data.get("timestamp", 0.0)
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)

    # --- Acceleration ---
    vec = data.get("vector", [0.0, 0.0, 0.0])
    if len(vec) != 3:
        raise ValueError(f"Vector must have 3 elements, got {len(vec)}")

    # Linear acceleration
    msg.accel.linear = Vector3(x=vec[0], y=vec[1], z=vec[2])

    # Angular acceleration (not provided → default 0)
    msg.accel.angular = Vector3(x=0.0, y=0.0, z=0.0)

    return msg

# chat wrote this function lol
# data is json not a string
def odom_from_json(data) -> Odometry:
    msg = Odometry()

    # --- Header ---
    msg.header = Header()
    msg.header.frame_id = data.get("frame_id", "")

    timestamp = data.get("timestamp", 0.0)
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)

    # --- Child frame ---
    msg.child_frame_id = data.get("child_frame_id", "")

    # --- Pose ---
    pose = data.get("pose", {})

    # Position
    pos = pose.get("position", [0.0, 0.0, 0.0])
    msg.pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])

    # Orientation
    ori = pose.get("orientation", {})
    msg.pose.pose.orientation = Quaternion(
        x=ori.get("x", 0.0),
        y=ori.get("y", 0.0),
        z=ori.get("z", 0.0),
        w=ori.get("w", 1.0),
    )

    # Pose covariance (36 elements)
    pose_cov = pose.get("covariance", [0.0] * 36)
    if len(pose_cov) == 36:
        msg.pose.covariance = pose_cov
    else:
        raise ValueError(f"Pose covariance must have 36 elements, got {len(pose_cov)}")

    # --- Twist ---
    twist = data.get("twist", {})

    # Linear velocity
    lin = twist.get("linear", [0.0, 0.0, 0.0])
    msg.twist.twist.linear = Vector3(x=lin[0], y=lin[1], z=lin[2])

    # Angular velocity
    ang = twist.get("angular", [0.0, 0.0, 0.0])
    msg.twist.twist.angular = Vector3(x=ang[0], y=ang[1], z=ang[2])

    # Twist covariance (36 elements)
    twist_cov = twist.get("covariance", [0.0] * 36)
    if len(twist_cov) == 36:
        msg.twist.covariance = twist_cov
    else:
        raise ValueError(f"Twist covariance must have 36 elements, got {len(twist_cov)}")

    return msg
