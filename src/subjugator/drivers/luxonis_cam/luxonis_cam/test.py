import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define sources
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_left = pipeline.create(dai.node.MonoCamera)
cam_right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

# Set camera properties
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

# Stereo depth configuration
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setSubpixel(True)

# Linking nodes
stereo.setInputResolution(1280, 720)
cam_left.out.link(stereo.left)
cam_right.out.link(stereo.right)

# Output queues
xout_color = pipeline.create(dai.node.XLinkOut)
xout_color.setStreamName("color")
cam_rgb.video.link(xout_color.input)

xout_left = pipeline.create(dai.node.XLinkOut)
xout_left.setStreamName("left")
cam_left.out.link(xout_left.input)

xout_right = pipeline.create(dai.node.XLinkOut)
xout_right.setStreamName("right")
cam_right.out.link(xout_right.input)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# Start device
with dai.Device(pipeline) as device:
    color_queue = device.getOutputQueue(name="color", maxSize=4, blocking=False)
    left_queue = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    right_queue = device.getOutputQueue(name="right", maxSize=4, blocking=False)
    depth_queue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    while True:
        color_frame = color_queue.get().getCvFrame() if color_queue.has() else None
        left_frame = left_queue.get().getCvFrame() if left_queue.has() else None
        right_frame = right_queue.get().getCvFrame() if right_queue.has() else None
        depth_frame = depth_queue.get().getCvFrame() if depth_queue.has() else None

        if color_frame is not None:
            cv2.imshow("Color", color_frame)
        if left_frame is not None:
            cv2.imshow("Left", left_frame)
        if right_frame is not None:
            cv2.imshow("Right", right_frame)
        if depth_frame is not None:
            cv2.imshow("Depth", depth_frame)

        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
