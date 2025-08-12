import cv2
import numpy as np
from cv2.typing import MatLike

from subjugator_centroids.centroid_finder import CentroidFinder


# example implementation of centroid abstract class, this one tracks green objects
class RedTracker(CentroidFinder):
    def __init__(self, topic_name: str):
        self.topic_name_: str = topic_name
        self.debug = False

    @property
    def topic_name(self) -> str:
        return self.topic_name_

    def find_centroid(self, frame: MatLike) -> tuple[int, int] | None:
        """
        Detect red areas in the image and return the centroid (x, y).
        Returns None if no red region is detected.
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red (wraps around the 0° hue boundary)
        lower_red1 = np.array([0, 140, 140])
        upper_red1 = np.array([8, 255, 255])

        lower_red2 = np.array([172, 140, 140])
        upper_red2 = np.array([180, 255, 255])

        # Create two masks and combine them
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        if self.debug:
            cv2.imshow("Red Mask", mask)
            cv2.waitKey(1)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Find the largest contour (ignore small ones)
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 50:
            return None

        # Compute centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return (cx, cy)

def test():
    gt = RedTracker("testing/rn/sry")
    gt.debug = True
    cap =cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("i hate you")
        gt.find_centroid(frame)
    
if __name__ == "__main__":
    test()
