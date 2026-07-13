from copy import copy
import math
from operator import itemgetter

import numpy as np
import admission as  adm
import gtsam
import transforms3d
from geometry_msgs.msg import Pose, Quaternion
from subjugator_msgs.srv import Servo

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

DEPTH = -0.8
TIMEOUT = 10

def yaw_from_odom(odom):
    yaw, _, _ = transforms3d.taitbryan.quat2euler([
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
    ])
    return yaw

def quat_from_odom(odom):
    q = Quaternion()
    (
        q.w,
        q.x,
        q.y,
        q.z
    ) = transforms3d.taitbryan.euler2quat(yaw_from_odom(odom), 0, 0)
    return q

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
    x, y = gtsam.Rot2(yaw_from_odom(odom)).rotate([x, y])
    return x, y

def absolute(track, odom):
    x, y = relative(track, odom)
    return (
        x + odom.pose.pose.position.x,
        y + odom.pose.pose.position.y
    )

def pick_track(yolo, odom, visited, visited_dets):
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
            if dist < 1 and angle < 0.5:
                break
            if det.id in visited_dets:
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
    visited_dets = set()

    backtrack = None
    visited_stack = []

    dropped = 0
    seen = 0
    lost = 0 # have track_id, but lost it
    homeless = 0 # no track_id

    while seen != 4:
        async for yolo, odom_msg in adm.Join(adm.yolo_down_sub, adm.odom_sub):
            if backtrack:
                if math.sqrt(
                    (odom_msg.pose.pose.position.x - backtrack[0]) ** 2 +
                    (odom_msg.pose.pose.position.y - backtrack[1]) ** 2
                ) < 0.1:
                    backtrack = None
                continue

            if yolo and odom:
                # pick centermost to track
                if track_id is None:
                    homeless += 1
                    track_id = pick_track(yolo, odom, visited, visited_dets)
                    if homeless == TIMEOUT:
                        backtrack = visited_stack.pop()
                        raise Exception("no place to backtrack")

                        goal = Pose()
                        goal.orientation = quat_from_odom(odom)
                        goal.position.x = backtrack[0]
                        goal.position.y = backtrack[1]
                        goal.position.z = DEPTH
                        adm.goal_pub.publish(goal)

                        homeless = 0
                else:
                    homeless = 0

                # find track
                tracks = [det for det in yolo.detections if det.id == track_id]
                if not len(tracks):
                    lost += 1
                    if lost == TIMEOUT:
                        track_id = pick_track(yolo, odom, visited, visited_dets)
                    elif lost == 2 * TIMEOUT:
                        raise Exception("lost")
                    continue
                else:
                    lost = 0

                track = tracks[0]

                x, y = relative(track, odom)
                dist = math.sqrt(x * x + y * y)

                goal = Pose()
                goal.orientation = quat_from_odom(odom)
                goal.position.x = x + odom.pose.pose.position.x
                goal.position.y = y + odom.pose.pose.position.y
                goal.position.z = DEPTH
                adm.goal_pub.publish(goal)

                if dist <= 0.1:
                    break

            elif odom_msg:
                odom = odom_msg

        # while yolo := adm.yolo_down_sub():
        #     break

        dropped += 1
        servo = Servo.Request()
        servo.angle = dropped
        adm.dropper_srv.call_async(servo)

        seen += 1
        visited.append((goal.position.x, goal.position.y))
        visited_dets.add(track_id)
        visited_stack.append((goal.position.x, goal.position.y))
        track_id = None

async def main():
    await dropper_over()

if __name__ == "__main__":
    adm.run(main())
