import numpy as np
import admission as  adm

"""
true front
    \
theta\+---(port)----+
------| camera view |
      +-(starboard)-+ true back

(skewed theta = 17.3 deg)
"""

ANG = 17.3 * math.pi / 180
ROT = np.array([
    [math.cos(ANG), math.sin(ANG)],
    [math.sin(ANG), -math.cos(ANG)]
])

async def dropper_over():
    odom = None
    track_id = None

    async for yolo, odom_msg := adm.Join(adm.yolo_down_sub, adm.odom_sub):
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

            # transform to odom
            """
                 z
            x -(+)
                |
                y
            """
            x = 320 - track.bbox.center.position.x
            y = track.bbox.center.position.y - 180
            x, y = ROT @ [x, y]

        elif odom_msg:
            odom = odom_msg

if __name__ == "__main__":
    adm.run(dropper_over())
