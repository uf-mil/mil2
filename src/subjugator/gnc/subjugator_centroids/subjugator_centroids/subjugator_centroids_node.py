# TODO: add the camera rotator to this and make it work w sub

# node which has a bunch of classes that implement the CentroidFinder abstract class, it feeds them images and publishes results

from typing import Dict, List

import cv2
import rclpy
from cv2.typing import MatLike
from rclpy.node import Node
from rclpy.publisher import Publisher
from subjugator_msgs.msg import Centroid

from subjugator_centroids.centroid_finder import CentroidFinder
from subjugator_centroids.green_tracker import GreenTracker


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
        value=(0, 0, 0),  # black
    )

    # Get new dimensions
    (h, w) = padded.shape[:2]
    center = (w // 2, h // 2)

    # Get rotation matrix
    M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)

    # Rotate the padded image
    rotated = cv2.warpAffine(padded, M, (w, h))
    return rotated


class SubjugatorCentroidsNode(Node):
    def __init__(self, trackers: List[CentroidFinder]):
        super().__init__("subjugator_centroids_node")
        self.pubs: Dict[str, Publisher] = {}
        self.trackers: List[CentroidFinder] = trackers

        # create publishers for each tracker
        for tracker in trackers:
            self.pubs[tracker.topic_name] = self.create_publisher(
                Centroid,
                tracker.topic_name,
                10,
            )

        # start camera
        self.cap = cv2.VideoCapture(2)  # Use your webcam
        if not self.cap.isOpened():
            self.get_logger().error("could not open camera")
            return

        self.search_for_centroids_forever()

    def search_for_centroids_forever(self):
        # literally just run this function forever
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("no frame from camera")
                break

            frame = rotate_front_cam(frame)

            for tracker in self.trackers:
                centroid = tracker.find_centroid(frame)
                if centroid is None:  # nothing to publish
                    continue

                height, width, _ = frame.shape
                topic_name = tracker.topic_name
                self.publish_centroid(
                    topic_name,
                    width,
                    height,
                    centroid[0],
                    centroid[1],
                )

    # image_width: images width in pixels
    # image_height: images height in pixels
    # cx: x position of the image centroid
    # cy: y position of the image centroid
    def publish_centroid(
        self,
        topic_name: str,
        image_width: int,
        image_height: int,
        cx: int,
        cy: int,
    ):
        c = Centroid()
        c.image_height = image_height
        c.image_width = image_width
        c.centroid_x = cx
        c.centroid_y = cy
        self.pubs[topic_name].publish(c)


def main():
    # here is where you create all of the things you want to track
    gt = GreenTracker("centroids/green")

    rclpy.init()
    n = SubjugatorCentroidsNode([gt])
    # rclpy.spin(n) node spins itself sry
    n.cap.release()  # free camera so cool
    rclpy.shutdown()


if __name__ == "__main__":
    main()
