# TODO: add the camera rotator to this and make it work w sub

# node which has a bunch of classes that implement the CentriodFinder abstract class, it feeds them images and publishes results

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from subjugator_msgs.msg import Centriod
from subjugator_centroids.centriod_finder import CentriodFinder
from subjugator_centroids.green_tracker import GreenTracker

import cv2
from typing import Dict, List

class SubjugatorCentroidsNode(Node):
    def __init__(self, trackers: List[CentriodFinder]):
        super().__init__("subjugator_centroids_node")
        self.pubs: Dict[str, Publisher] = {}
        self.trackers: List[CentriodFinder] = trackers

        # create publishers for each tracker
        for tracker in trackers:
            self.pubs[tracker.topic_name] = self.create_publisher(Centriod, tracker.topic_name, 10)

        # start camera
        self.cap = cv2.VideoCapture(0)  # Use your webcam
        if not self.cap.isOpened():
            self.get_logger().error("could not open camera")
            return

        self.search_for_centriods_forever()

    def search_for_centriods_forever(self): 
        # literally just run this function forever
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("no frame from camera")
                break

            for tracker in self.trackers:
                centroid = tracker.find_centriod(frame)
                if centroid is None: # nothing to publish
                    continue

                height, width, _ = frame.shape
                topic_name = tracker.topic_name
                self.publish_centriod(topic_name, width, height, centroid[0], centroid[1])

        

    # image_width: images width in pixels
    # image_height: images height in pixels
    # cx: x position of the image centriod
    # cy: y position of the image centriod
    def publish_centriod(self, topic_name: str,
                               image_width: int,
                               image_height: int,
                               cx: int,
                               cy: int):
        c = Centriod()
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
    n.cap.release() # free camera so cool
    rclpy.shutdown()

if __name__ == "__main__":
    main()
