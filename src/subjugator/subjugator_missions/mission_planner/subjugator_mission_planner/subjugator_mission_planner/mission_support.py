import math

import numpy as np
import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node


# Computes the euclydian distance between two poses.
# The distance between position and orientation are computed separately to ensure combined movee + rotate tasks are fully completed
def check_at_goal_pose(
    self,
    currentPose,
    goalPose,
    acceptablePosDist=0.05,
    acceptableOrientDist=0.1,
):
    x_dist = currentPose.position.x - goalPose.position.x
    y_dist = currentPose.position.y - goalPose.position.y
    z_dist = currentPose.position.z - goalPose.position.z

    i_dist = currentPose.orientation.x - goalPose.orientation.x
    j_dist = currentPose.orientation.y - goalPose.orientation.y
    k_dist = currentPose.orientation.z - goalPose.orientation.z
    w_dist = currentPose.orientation.w - goalPose.orientation.w

    distance_to_goal = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
    orientation_to_goal = math.sqrt(i_dist**2 + j_dist**2 + k_dist**2 + w_dist**2)
    return (
        distance_to_goal < acceptablePosDist
        and orientation_to_goal < acceptableOrientDist
    )


# Finds the closest point between two lines with least squares regression
def closest_points_between_lines(a1, v1, b1, v2):
    """
    Finds the closest points on two lines defined by:
    - Line 1: a1 + t * v1
    - Line 2: b1 + s * v2

    Returns:
        p1_closest: closest point on line 1
        p2_closest: closest point on line 2
        midpoint: the average of the two (midpoint between the lines)
    """
    # Ensure numpy arrays
    a1 = np.array(a1, dtype=np.float64)
    v1 = np.array(v1, dtype=np.float64)
    b1 = np.array(b1, dtype=np.float64)
    v2 = np.array(v2, dtype=np.float64)

    # Define some dot products
    r = a1 - b1
    v1_dot_v1 = np.dot(v1, v1)
    v2_dot_v2 = np.dot(v2, v2)
    v1_dot_v2 = np.dot(v1, v2)
    v1_dot_r = np.dot(v1, r)
    v2_dot_r = np.dot(v2, r)

    denom = v1_dot_v1 * v2_dot_v2 - v1_dot_v2**2

    # If denom is zero, lines are parallel — handle gracefully
    if np.isclose(denom, 0.0):
        # Pick arbitrary point on line 1, project onto line 2
        t = 0
        s = v2_dot_r / v2_dot_v2
    else:
        t = (v1_dot_v2 * v2_dot_r - v2_dot_v2 * v1_dot_r) / denom
        s = (v1_dot_v2 * t + v2_dot_r) / v2_dot_v2

    # Closest points
    p1_closest = a1 + t * v1
    p2_closest = b1 + s * v2
    midpoint = (p1_closest + p2_closest) / 2.0

    return midpoint


def quaternion_to_forward_vector(x, y, z, w):
    """
    Convert a quaternion into a unit vector representing the forward direction.
    Assumes the local forward direction is +X.

    Args:
        x, y, z, w: Quaternion components

    Returns:
        np.array of shape (3,) representing unit forward vector in world coords
    """
    # Convert quaternion to rotation matrix
    # Rotation matrix formula from quaternion
    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ],
    )

    # Local forward vector (pointing along +X)
    forward_local = np.array([1, 0, 0])

    # Rotate forward vector by quaternion rotation matrix
    forward_world = R @ forward_local

    # Normalize to get unit vector
    forward_unit = forward_world / np.linalg.norm(forward_world)

    return forward_unit


# The ActionUser class allows tasks to be called within other tasks (written by Joe)
class ActionUser:

    def __init__(self, node: Node, action_type, action_name: str):
        self.node = node
        self.ac = ActionClient(node, action_type, action_name)

    # 0 timeout seconds implies no timeout
    # TODO rn there is nothing for timeout_sec, would be a great first issue for someone :) (use self.node.get_clock().now())
    # _=0 should be timeout_sec = 0
    def send_goal_and_block_until_done(self, goal, _=0):
        future = self.ac.send_goal_async(goal)  # rn no feedback
        running = True
        while running:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if future.done():
                result = future.result()
                result_future = result.get_result_async()

                while not result_future.done():  # TODO this could be better
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                running = False

    def send_goal_and_return_future(self, goal):
        return self.ac.send_goal_async(goal)


"""
Example usage:

Creating client (usually in init):

    self.yaw_tracker_client = ActionUser(self, YawTracker, "yawtracker")

Sending goal to client:

    self.move_client.send_goal_and_block_until_done(goal)


"""
