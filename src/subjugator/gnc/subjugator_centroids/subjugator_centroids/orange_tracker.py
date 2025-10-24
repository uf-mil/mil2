import cv2
import numpy as np
from cv2.typing import MatLike

from subjugator_centroids.centroid_finder import CentroidFinder


class OrangeTracker(CentroidFinder):
    def __init__(self, topic_name: str):
        self.topic_name_: str = topic_name

    @property
    def topic_name(self) -> str:
        return self.topic_name_

    def find_centroid(self, frame: MatLike) -> tuple[int, int] | None:
        """
        Detect orange areas in the image and return the centroid (x, y).
        Returns None if no orange region is detected.
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower and upper bounds for orange in HSV
        lower_orange = np.array([10, 100, 20])
        upper_orange = np.array([25, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

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
