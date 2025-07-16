# TODO: add the camera rotator to this and make it work w sub

import rclpy
from rclpy.node import Node
from subjugator_msgs.msg import Centriod
from subjugator_centroids.green_tracker import get_lime_green_centroid

import cv2

class SubjugatorCentroidsNode(Node):
    def __init__(self):
        super().__init__("subjugator_centroids_node")

        # create publisher
        self.centriod_pub = self.create_publisher(Centriod, "/centriods/green", 10)

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

            centroid, _ = get_lime_green_centroid(frame)
            if centroid is None: # nothing to publish
                continue

            height, width, _ = frame.shape
            self.publish_centriod(width, height, centroid[0], centroid[1])

        

    # image_width: images width in pixels
    # image_height: images height in pixels
    # cx: x position of the image centriod
    # cy: y position of the image centriod
    def publish_centriod(self, image_width: int,
                               image_height: int,
                               cx: int,
                               cy: int):
        c = Centriod()
        c.image_height = image_height
        c.image_width = image_width
        c.centroid_x = cx
        c.centroid_y = cy
        self.centriod_pub.publish(c)

def main():
    rclpy.init()
    n = SubjugatorCentroidsNode()
    # rclpy.spin(n) node spins itself sry
    n.cap.release()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
