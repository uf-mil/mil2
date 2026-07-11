from copy import copy
import math
from operator import itemgetter

import numpy as np
import admission as  adm
import gtsam
import transforms3d
from geometry_msgs.msg import Pose

"""
true front
    \\
theta\\+---(port)----+
------ | camera view |
       +-(starboard)-+ true back

(skewed theta = 17.3 deg)
"""

ROT = gtsam.Rot2(17.3 * math.pi / 180)
SCALE = math.tan(40 * math.pi / 180) / 320

def relative(track, odom):
    # transform camera to base_link (relative)
    """
          z
    x--(+)
        |
        y
    """
    x, y = ROT.rotate(np.array([
        320 - track.bbox.center.position.x,
        track.bbox.center.position.y - 180
    ]) * SCALE)

    # transform base_link to odom
    yaw, _, _ = transforms3d.taitbryan.quat2euler([
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
    ])
    x, y = gtsam.Rot2(yaw).rotate([x, y])
    return x, y, yaw

def absolute(track, odom):
    x, y, _ = relative(track, odom)
    return (
        x + odom.pose.pose.position.x,
        y + odom.pose.pose.position.y
    )

def pick_track(yolo, odom, visited):
    dets = []
    for det in yolo.detections:
        x, y = absolute(det, odom)
        ox = odom.pose.pose.position.x
        oy = odom.pose.pose.position.y
        for vx, vy in visited:
            angle = math.fabs(math.remainder(
                math.atan2(
                    vy - oy,
                    vx - ox
                ) - math.atan2(
                    y - oy,
                    x - ox
                ),
                math.tau
            ))
            dist = math.sqrt((x - vx) ** 2 + (y - vy) ** 2)
            if dist < 0.25  or angle > 1:
                break
        else:
            # pick closest to camera
            cx = det.bbox.center.position.x - 320
            cy = det.bbox.center.position.y - 180
            cdist = math.sqrt(cx * cx + cy * cy)

            dets.append((det.id, cdist))

    if not len(dets):
        return None

    return sorted(dets, key=itemgetter(1))[0][0]

async def dropper_over():
    odom = None
    track_id = None
    visited = []

    dropped = 0
    lost = 0

    while dropped != 4:
        async for yolo, odom_msg in adm.Join(adm.yolo_down_sub, adm.odom_sub):
            if yolo and odom:
                # pick centermost to track
                if track_id is None:
                    track_id = pick_track(yolo, odom, visited)

                # find track
                tracks = [det for det in yolo.detections if det.id == track_id]
                if not len(tracks):
                    lost += 1
                    if lost == 10:
                        lost = 0
                        track_id = pick_track(yolo, odom, visited)
                    continue

                track = tracks[0]

                x, y, yaw = relative(track, odom)
                dist = math.sqrt(x * x + y * y)

                goal = Pose()
                (
                    goal.orientation.w,
                    goal.orientation.x,
                    goal.orientation.y,
                    goal.orientation.z
                ) = transforms3d.taitbryan.euler2quat(yaw, 0, 0)

                goal.position.x = x + odom.pose.pose.position.x
                goal.position.y = y + odom.pose.pose.position.y
                goal.position.z = -0.5
                adm.goal_pub.publish(goal)

                if dist <= 0.1:
                    break

            elif odom_msg:
                odom = odom_msg

        print("drop")
        dropped += 1
        visited.append((goal.position.x, goal.position.y))
        track_id = None

if __name__ == "__main__":
    adm.run(dropper_over())
