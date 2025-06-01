import depthai as dai
import cv2

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

# Start pipeline on device
with dai.Device(pipeline) as device:
    video_queue = device.getOutputQueue(name="video")

    while True:
        in_frame = video_queue.get()
        frame = in_frame.getCvFrame()

        cv2.imshow("DepthAI Camera", frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
