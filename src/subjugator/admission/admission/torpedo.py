import dataclasses
import math
from operator import itemgetter

import numpy as np

import gtsam
import admission as adm
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

def odom_pose(odom):
    o_p = odom.pose.pose.position
    o_q = odom.pose.pose.orientation
    pose = gtsam.Pose3(
        gtsam.Rot3(o_q.w, o_q.x, o_q.y, o_q.z),
        [o_p.x, o_p.y, o_p.z]
    )
    noise = gtsam.noiseModel.Gaussian.Covariance(
        odom.pose.covariance.reshape(6, 6)
    )
    return pose, noise

async def torpedo():
    while yolo := adm.Join():
        for det in yolo.detections:
            print(det)

if __name__ == "__main__":
    adm.run(torpedo())
