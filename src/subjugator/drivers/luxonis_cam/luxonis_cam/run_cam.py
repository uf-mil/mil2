import threading
import time

import depthai as dai
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class LuxonisCamera(Node):
    def __init__(self):
        super().__init__("luxonis_camera")
        self.rgb_pub = self.create_publisher(Image, "cam_rgb", 10)
        self.mono_pub = self.create_publisher(Image, "cam_mono", 10)
        self.depth_pub = self.create_publisher(Image, "cam_depth", 10)

        self.__pipeline = dai.Pipeline()

        self.cam_rgb = self.__pipeline.create(dai.node.ColorCamera)
        self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        self.cam_left = self.__pipeline.create(dai.node.MonoCamera)
        self.cam_right = self.__pipeline.create(dai.node.MonoCamera)
        self.stereo = self.__pipeline.create(dai.node.StereoDepth)

        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        self.cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        self.stereo.setLeftRightCheck(True)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        self.stereo.setSubpixel(False)
        self.stereo.setRectifyEdgeFillColor(0)
        self.stereo.setInputResolution(1280, 720)

        self.cam_left.out.link(self.stereo.left)
        self.cam_right.out.link(self.stereo.right)

        self.xout_color = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_color.setStreamName("cam_rgb")
        self.cam_rgb.video.link(self.xout_color.input)

        self.xout_left = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_left.setStreamName("left")
        self.cam_left.out.link(self.xout_left.input)

        self.xout_right = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_right.setStreamName("right")
        self.cam_right.out.link(self.xout_right.input)

        self.xout_depth = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_depth.setStreamName("cam_depth")
        self.stereo.depth.link(self.xout_depth.input)

        self.bridge = CvBridge()

        # Launch run in a background thread so ROS can spin separately
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        with dai.Device(self.__pipeline) as device:
            color_queue = device.getOutputQueue("cam_rgb", maxSize=4, blocking=False)
            mono_queue = device.getOutputQueue("left", maxSize=4, blocking=False)
            depth_queue = device.getOutputQueue("cam_depth", maxSize=4, blocking=False)

            while rclpy.ok():
                color_frame = (
                    color_queue.get().getCvFrame() if color_queue.has() else None
                )
                mono_frame = mono_queue.get().getCvFrame() if mono_queue.has() else None
                depth_frame = (
                    depth_queue.get().getCvFrame() if depth_queue.has() else None
                )

                # print(color_frame is not None, mono_frame is not None, depth_frame is not None)

                if color_frame is not None:
                    print("rgb")
                    msg = self.bridge.cv2_to_imgmsg(color_frame, "bgr8")
                    self.rgb_pub.publish(msg)
                if mono_frame is not None:
                    print("mono")
                    msg = self.bridge.cv2_to_imgmsg(mono_frame, "mono8")
                    self.mono_pub.publish(msg)
                if depth_frame is not None:
                    print("depth!")
                    msg = self.bridge.cv2_to_imgmsg(depth_frame, "mono16")
                    self.depth_pub.publish(msg)

                time.sleep(0.03)


def main(args=None):
    rclpy.init(args=args)
    cam = LuxonisCamera()
    rclpy.spin(cam)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
