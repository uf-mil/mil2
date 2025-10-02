import cv2

# Open the video device (usually /dev/video0)
cap = cv2.VideoCapture(0)  # 0 = /dev/video0

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open /dev/video0")
    exit()

# Set frame width and height (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Press 'q' to quit.")

# Loop to read and display frames
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    cv2.imshow("USB Camera - /dev/video0", frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
