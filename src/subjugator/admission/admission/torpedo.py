import cv2
import numpy as np
import transforms3d

import admission as adm
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

"""
0 survey&repair_small
1 survey&repair_large
2 search&rescue_small
3 search&rescue_large
4 torpedoTarget
+-------+
|   o (survey)
| O (survey)
|   o O |
(search x2)
"""

reference = [
    (185, 68),
    (58, 148),
    (180, 293),
    (307, 308),
    (180, 180) # 0, 0
]
reference = [(x + (640 - 360) / 2, y) for x, y in reference]

cal = np.array([
    381.361, 0, 320,
    0, 381.361, 180,
    0, 0, 1
])

r_in_cv = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0]
])

cv_in_r = r_in_cv.T

def mark(i, pos, rot):
    marker = Marker()
    marker.id = i
    marker.header.frame_id = "base_link"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.color.a = marker.color.g = 1.0

    marker.scale.x = 1.0
    marker.scale.y = marker.scale.z = 0.1
    (
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z
    ) = pos # r_in_cv @ pos

    try:
        (
            marker.pose.orientation.w,
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
        ) = transforms3d.quaternions.mat2quat(rot) # (r_in_cv @ rot) @ cv_in_r)
    except np.linalg.LinAlgError:
        return
    adm.marker_pub.publish(marker)


async def torpedo():
    while yolo := await adm.yolo_sub():
        dets = [None] * 5
        for det in yolo.detections:
            # if det.class_id == 4: continue
            pos = det.bbox.center.position
            if dets[det.class_id] is None or pos.y > dets[det.class_id][1]:
                dets[det.class_id] = pos.x, pos.y

        print(dets)

        refs = [reference[i] for i in range(len(dets)) if dets[i] is not None]
        dets = [det for det in dets if det is not None]

        if len(dets) < 4: continue

        # dets = np.array(dets) - dets[4]
        homo, _ = cv2.findHomography(np.array(dets), np.array(refs))
        # print(homo)

        [nsols, rots, trans, norms] = cv2.decomposeHomographyMat(homo, cal)

        i = 0
        for rot, tran in zip(rots, trans):
            # if tran[2] <= 0: continue
            mark(i, tran.T[0], rot)
            i += 1
        # for rot, trans, norm in sols[1:]:
        #     print(rot)
        # print("=" * 10)

if __name__ == "__main__":
    adm.run(torpedo())
