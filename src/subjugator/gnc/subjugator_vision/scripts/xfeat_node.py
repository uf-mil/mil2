import cv2
import torch
import numpy as np
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray

xfeat = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained = True, top_k = 4096)

def warp_points(points, H, x_offset = 0):
    points_np = np.array(points, dtype='float32').reshape(-1,1,2)

    warped_points_np = cv2.perspectiveTransform(points_np, H).reshape(-1, 2)
    # warped_points_np[:, 0] += x_offset
    warped_points = warped_points_np.astype(int).tolist()
    
    return warped_points

def draw_quad(frame, point_list):
    for i in range(3):
        cv2.line(frame, tuple(point_list[i]), tuple(point_list[i + 1]), (0,255,0), 3)
    cv2.line(frame, tuple(point_list[3]), tuple(point_list[0]), (0,255,0), 3)


class XFeat(Node):
    def __init__(self):
        super().__init__("xfeat")

        # calculate reference
        img = np.array(PILImage.open("reference.png").resize((360, 360)))
        reference = xfeat.detectAndCompute(img, top_k = 4096)[0]
        kpts1, self.descs1 = reference['keypoints'], reference['descriptors']
        self.kpts1 = kpts1.cpu().numpy() + [(640 - 360) / 2, 0]

        self.create_subscription(DetectionArray, "/yolo/detections",
                                 self.yolo_cb, 10)
        self.create_subscription(Image, "/front_cam/image_raw",
                                 self.img_cb, 10)
        self.debug_pub = self.create_publisher(Image, "/xfeat/debug", 10)
        self.img = None

    def img_cb(self, img):
        self.img = img
        # self.yolo_cb(None, use_yolo=False)

    def yolo_cb(self, yolo, *, use_yolo=True):
        if not self.img:
            return

        if use_yolo:
            detections = min(
                [
                    det
                    for det in yolo.detections
                    if det.class_id == 4 # torpedo
                ],
                key=lambda d: d.bbox.center.position.y
            )

            try:
                torpedo = detections[0]
            except IndexError:
                return

        im = np.ndarray(
            shape=(
                self.img.height,
                self.img.width,
                3
            ),
            dtype=np.uint8,
            buffer=self.img.data
        )

        if use_yolo:
            bb = torpedo.bbox
            x1 = math.floor(bb.center.position.x - bb.size.x / 2)
            y1 = math.floor(bb.center.position.y - bb.size.y / 2)
            x2 = math.ceil(bb.center.position.x + bb.size.x / 2)
            y2 = math.ceil(bb.center.position.y + bb.size.y / 2)

            segment = np.flip(
                im[y1:y2, x1:x2],
                axis=-1
            ).copy()
        else:
            segment = np.flip(im, axis=-1).copy()
            x1 = y1 = 0

        # run xfeat
        current = xfeat.detectAndCompute(segment, top_k = 4096)[0]
        kpts2, descs2 = current['keypoints'], current['descriptors']

        idx0, idx1 = xfeat.match(self.descs1, descs2, 0.82)
        points1 = self.kpts1[idx0.cpu()]
        points2 = kpts2[idx1].cpu().numpy()

        if len(points1) <= 10 or len(points2) <= 10:
            return

        H, inliers = cv2.findHomography(
            points1, points2 + [x1, y1],
            cv2.USAC_MAGSAC
        )

        pts = warp_points([[(640 - 360) / 2, 0], [360 + (640 - 360) / 2, 0],
                           [360 + (640 - 360) / 2, 360], [(640 - 360) / 2, 360]], H)

        # debug
        draw_quad(im, pts)

        dbg_img = Image()
        dbg_img.header = self.img.header
        dbg_img.data = im.tobytes()
        dbg_img.height = self.img.height
        dbg_img.width = self.img.width
        dbg_img.encoding = self.img.encoding
        dbg_img.step = self.img.step
        self.debug_pub.publish(dbg_img)

def main():
    rclpy.init()
    rclpy.spin(XFeat())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
