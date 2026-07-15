import math

import cv2
import numpy as np
import torch
from PIL import Image as PILImage
from sensor_msgs.msg import Image

from admission import adm

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

xfeat = torch.hub.load(
    "verlab/accelerated_features",
    "XFeat",
    pretrained=True,
    top_k=4096,
)

# Simple inference with batch sz = 1
img = np.array(PILImage.open("my_photo-2a.jpg").resize((360, 360)))
reference = xfeat.detectAndCompute(img, top_k=4096)[0]
kpts1, descs1 = reference["keypoints"], reference["descriptors"]
kpts1 = kpts1.cpu().numpy() + np.array([(640 - 360) / 2, 0])


def warp_points(points, H, x_offset=0):
    points_np = np.array(points, dtype="float32").reshape(-1, 1, 2)

    warped_points_np = cv2.perspectiveTransform(points_np, H).reshape(-1, 2)
    # warped_points_np[:, 0] += x_offset
    warped_points = warped_points_np.astype(int).tolist()

    return warped_points


def draw_quad(frame, point_list):
    for i in range(3):
        cv2.line(frame, tuple(point_list[i]), tuple(point_list[i + 1]), (0, 255, 0), 3)
    cv2.line(frame, tuple(point_list[3]), tuple(point_list[0]), (0, 255, 0), 3)


async def torpedo():
    last_img = None
    async for img, yolo in adm.Join(adm.frontcam_sub, adm.yolo_sub):
        if img:
            last_img = img
            continue

        if not last_img:
            continue
        detections = sorted(
            [det for det in yolo.detections if det.class_id == 4],  # torpedo
            key=lambda d: d.bbox.center.position.y,
        )

        try:
            torpedo = detections[0]
        except IndexError:
            continue

        im = np.ndarray(
            shape=(last_img.height, last_img.width, 3),
            dtype=np.uint8,
            buffer=last_img.data,
        )

        bb = torpedo.bbox
        x1 = math.floor(bb.center.position.x - bb.size.x / 2)
        y1 = math.floor(bb.center.position.y - bb.size.y / 2)
        x2 = math.ceil(bb.center.position.x + bb.size.x / 2)
        y2 = math.ceil(bb.center.position.y + bb.size.y / 2)

        segment = np.flip(im[y1:y2, x1:x2], axis=-1).copy()

        current = xfeat.detectAndCompute(segment, top_k=4096)[0]
        kpts2, descs2 = current["keypoints"], current["descriptors"]

        idx0, idx1 = xfeat.match(descs1, descs2, 0.82)
        points1 = kpts1[idx0.cpu()]
        points2 = kpts2[idx1].cpu().numpy()

        if len(points1) <= 10 or len(points2) <= 10:
            continue

        H, inliers = cv2.findHomography(
            points1,
            points2 + np.array([x1, y1]),
            cv2.USAC_MAGSAC,
        )

        pts = warp_points(
            [
                [(640 - 360) / 2, 0],
                [360 + (640 - 360) / 2, 0],
                [360 + (640 - 360) / 2, 360],
                [(640 - 360) / 2, 360],
            ],
            H,
        )
        draw_quad(im, pts)

        dbg_img = Image()
        dbg_img.header = last_img.header
        dbg_img.data = im.tobytes()
        dbg_img.height = last_img.height
        dbg_img.width = last_img.width
        dbg_img.encoding = last_img.encoding
        dbg_img.step = last_img.step
        adm.debug_pub.publish(dbg_img)


if __name__ == "__main__":
    adm.run(torpedo())
