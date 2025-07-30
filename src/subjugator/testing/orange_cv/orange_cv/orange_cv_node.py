import cv2
import numpy as np
import rclpy
from cv2.typing import MatLike
from rclpy.node import Node
from subjugator_msgs.msg import Orange


class OrangeCV(Node):
    def __init__(self):
        super().__init__("orange_cv")
        self.orange_pub = self.create_publisher(Orange, "centroids/orange", 10)

        # TODO real camera id
        self.cap = cv2.VideoCapture(0)

        self.timer_ = self.create_timer(33 / 1000, self.get_and_pub_frame)

    def get_and_pub_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            return

        bbox, centroid, _ = detect_bright_orange_object(frame)
        if bbox is None or centroid is None:
            return

        _, _, width, height = bbox
        cx, cy = centroid

        msg = Orange()
        msg.height = float(height)
        msg.width = float(width)
        msg.cx = float(cx)
        msg.cy = float(cy)

        self.orange_pub.publish(msg)


def main():
    rclpy.init()
    n = OrangeCV()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# TODO rotate ts
def detect_bright_orange_object(image: MatLike):
    """
    Detects the largest bright/neon orange object in an image.

    Args:
        image: The input image in BGR format

    Returns:
        A tuple containing:
        - bbox: Single bounding box as (x, y, width, height) or None
        - centroid: Single centroid as (x, y) or None
        - mask: Binary mask for the detected orange object or None
    """
    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define range for bright/neon orange color - more restrictive
    # Bright orange is around 15 in the H channel with high S and V
    lower_orange = np.array([10, 180, 180])  # Higher saturation and value
    upper_orange = np.array([25, 255, 255])

    # Create a mask for orange colored objects
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Apply morphological operations to remove noise and fill holes
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If no contours found, return None
    if not contours:
        return None, None, None

    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)

    # Check if the area is too small (noise)
    area = cv2.contourArea(largest_contour)
    if area < 300:  # Higher threshold to filter out small noise
        return None, None, None

    # Get bounding box
    x, y, w, h = cv2.boundingRect(largest_contour)
    bbox = (x, y, w, h)

    # Calculate centroid
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = x + w // 2, y + h // 2
    centroid = (cx, cy)

    # Create mask for this contour
    obj_mask = np.zeros_like(mask)
    cv2.drawContours(obj_mask, [largest_contour], -1, 255, -1)

    return bbox, centroid, obj_mask
