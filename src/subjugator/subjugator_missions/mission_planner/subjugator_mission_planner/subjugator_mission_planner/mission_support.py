import math

# Commonly used functions throughout the mission planner


def check_at_goal_pose(self, currentPose, goalPose, acceptableDist=0.05):
    x_dist = currentPose.position.x - goalPose.position.x
    y_dist = currentPose.position.y - goalPose.position.y
    z_dist = currentPose.position.z - goalPose.position.z

    i_dist = currentPose.orientation.x - goalPose.orientation.x
    j_dist = currentPose.orientation.y - goalPose.orientation.y
    k_dist = currentPose.orientation.z - goalPose.orientation.z
    w_dist = currentPose.orientation.w - goalPose.orientation.w

    distance_to_goal = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
    orientation_to_goal = math.sqrt(i_dist**2 + j_dist**2 + k_dist**2 + w_dist**2)
    return distance_to_goal < acceptableDist and orientation_to_goal < 0.1
