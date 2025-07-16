from subjugator_centroids.centriod_finder import CentriodFinder
import cv2
from cv2.typing import MatLike
import numpy as np

# example implementation of centriod abstract class, this one tracks green objects
class GreenTracker(CentriodFinder):
    def __init__(self, topic_name: str):
        self.topic_name_: str = topic_name

    @property
    def topic_name(self) -> str:
        return self.topic_name_

    def find_centriod(self, frame: MatLike) -> tuple[int, int] | None:
        """
        Detect lime green areas in the image and return the centroid (x, y).
        Returns None if no lime green region is detected.
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for lime green
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Find the largest contour
        largest = max(contours, key=cv2.contourArea)

        # Compute centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
        
