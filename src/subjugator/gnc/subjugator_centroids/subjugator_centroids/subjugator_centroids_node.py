# TODO: add the camera rotator to this and make it work w sub

# node which has a bunch of classes that implement the CentroidFinder abstract class, it feeds them images and publishes results

from typing import List

import cv2
import rclpy
from cv2.typing import MatLike
from rclpy.node import Node

from subjugator_centroids.centroid_finder import CentroidFinder
from subjugator_centroids.green_tracker import GreenTracker
from subjugator_centroids.red_tracker import RedTracker 

from mil_msgs.msg import PerceptionTarget

def rotate_front_cam(frame: MatLike) -> MatLike:
    PADDING = 100
    rotation_angle = 137
    # Add black padding around the image
    padded = cv2.copyMakeBorder(
        frame,
        top=PADDING,
        bottom=PADDING,
        left=PADDING,
        right=PADDING,
        borderType=cv2.BORDER_CONSTANT,
        value=(0, 0, 0), # black
    )
    (h, w) = padded.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)
    rotated = cv2.warpAffine(padded, M, (w, h))
    return rotated


class SubjugatorCentroidsNode(Node):
    def __init__(self, trackers: List[CentroidFinder]):
        super().__init__("subjugator_centroids_node")
        self.trackers = trackers
        self.percep_pub = self.create_publisher(PerceptionTarget, "/perception/targets", 10)
        # start camera
        cam_path = "/dev/v4l/by-id/usb-Chicony_Tech._Inc._Dell_Webcam_WB7022_4962D17A78D6-video-index0"
        self.cap = cv2.VideoCapture(cam_path)
        if not self.cap.isOpened():
            self.get_logger().error("could not open camera")
            return
        self.search_for_centroids_forever()

    def search_for_centroids_forever(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("no frame from camera")
                continue
            frame = rotate_front_cam(frame)
            h, w, _ = frame.shape
            for tracker in self.trackers:
                centroid = tracker.find_centroid(frame)
                if centroid is None:
                    continue
                cx, cy = centroid
                label = tracker.topic_name.split("/")[-1]
                self.publish_centroid_common(label, w, h, cx, cy)

    def publish_centroid_common(self, label, img_w, img_h, cx, cy):
        msg = PerceptionTarget()
        msg.label = label
        msg.cx, msg.cy = float(cx), float(cy)
        msg.width = 0.0
        msg.height = 0.0
        msg.confidence = 1.0
        msg.stamp = self.get_clock().now().to_msg()
        self.percep_pub.publish(msg)

def main():
    gt = GreenTracker("centroids/green")
    rt = RedTracker("centroids/red")
    ot = OrangeTracker("centroids/orange")
    rclpy.init()
    SubjugatorCentroidsNode([gt, rt, ot])
    rclpy.shutdown()

if __name__ == "__main__":
    main()
