# this file isn't used and it's just an example of how to use cv2 to find centroids

import cv2
import numpy as np


def get_lime_green_centroid(image):
    """
    Detect lime green areas in the image and return the centroid (x, y).
    Returns None if no lime green region is detected.
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV range for lime green
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    # Find the largest contour
    largest = max(contours, key=cv2.contourArea)

    # Compute centroid
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy), mask


def display_debug_image(image, mask, centroid):
    """
    Displays the original image, the binary mask, and overlays the centroid (if found).
    """
    # Clone the original image
    display_img = image.copy()

    # Overlay centroid if it exists
    if centroid:
        cv2.circle(display_img, centroid, 5, (0, 0, 255), -1)
        cv2.putText(
            display_img,
            f"Centroid: {centroid}",
            (centroid[0] + 10, centroid[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )

    # Show images
    cv2.imshow("Lime Green Detection", display_img)
    cv2.imshow("Mask", mask)


def main():
    cam_path = "/dev/v4l/by-id/usb-Chicony_Tech._Inc._Dell_Webcam_WB7022_4962D17A78D6-video-index0"
    cap = cv2.VideoCapture(cam_path)

    if not cap.isOpened():
        print("Cannot open camera")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        centroid, mask = get_lime_green_centroid(frame)
        display_debug_image(frame, mask, centroid)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
