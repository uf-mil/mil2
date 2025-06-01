import cv2
import numpy as np
import depthai as dai

# Create DepthAI pipeline
pipeline = dai.Pipeline()

# Create a color camera node
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setFps(30)

# Create an output node for the RGB preview
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam_rgb.preview.link(xout.input)

def nothing(x):
    pass

# Create window and trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 10, 179, nothing)
cv2.createTrackbar("H Max", "Trackbars", 25, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

with dai.Device(pipeline) as device:
    video_queue = device.getOutputQueue(name="video")

    while True:
        in_frame = video_queue.get()
        frame = in_frame.getCvFrame()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Read trackbar positions
        h_min = cv2.getTrackbarPos("H Min", "Trackbars")
        h_max = cv2.getTrackbarPos("H Max", "Trackbars")
        s_min = cv2.getTrackbarPos("S Min", "Trackbars")
        s_max = cv2.getTrackbarPos("S Max", "Trackbars")
        v_min = cv2.getTrackbarPos("V Min", "Trackbars")
        v_max = cv2.getTrackbarPos("V Max", "Trackbars")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(hsv, lower, upper)

        # Optional: clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Contour detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

        if filtered:
            largest = max(filtered, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"Orange blob center: ({cx}, {cy})")
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.drawContours(frame, [largest], -1, (0, 255, 255), 2)

        # Show images
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
