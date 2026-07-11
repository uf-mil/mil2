from copy import copy
import math

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

async def dropper_over():
    odom = None
    track_id = None

    async for yolo, odom_msg in adm.Join(adm.yolo_down_sub, adm.odom_sub):
        if yolo and odom:
            # pick centermost to track
            if track_id is None:
                dets = sorted([
                    (
                        det.id,
                        det.bbox.center.position.x,
                        det.bbox.center.position.y,
                    )
                    for det in yolo.detections
                ])
                if not len(dets):
                    continue
                track_id = dets[0][0]

            # find track
            tracks = [det for det in yolo.detections if det.id == track_id]
            if not len(tracks):
                continue
            track = tracks[0]

            # transform camera to base_link
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
            x += odom.pose.pose.position.x
            y += odom.pose.pose.position.y

            goal = Pose()
            (
                goal.orientation.w,
                goal.orientation.x,
                goal.orientation.y,
                goal.orientation.z
            ) = transforms3d.taitbryan.euler2quat(yaw, 0, 0)
            goal.position.x = x
            goal.position.y = y
            goal.position.z = -0.5
            adm.goal_pub.publish(goal)

        elif odom_msg:
            odom = odom_msg

if __name__ == "__main__":
    adm.run(dropper_over())
