import threading

import depthai as dai
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class LuxonisCamera(Node):
    def __init__(self):
        super().__init__("luxonis_camera")
        self.rgb_pub = self.create_publisher(Image, "cam_rgb", 10)
        self.left_pub = self.create_publisher(Image, "cam_left", 10)
        self.right_pub = self.create_publisher(Image, "cam_right", 10)
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
            color_queue = device.getOutputQueue(
                name="cam_rgb",
                maxSize=4,
                blocking=False,
            )
            left_queue = device.getOutputQueue(name="left", maxSize=4, blocking=False)
            right_queue = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            depth_queue = device.getOutputQueue(
                name="cam_depth",
                maxSize=4,
                blocking=False,
            )

            while rclpy.ok():
                color_frame = (
                    color_queue.get().getCvFrame() if color_queue.has() else None
                )
                left_frame = left_queue.get().getCvFrame() if left_queue.has() else None
                right_frame = (
                    right_queue.get().getCvFrame() if right_queue.has() else None
                )
                depth_frame = (
                    depth_queue.get().getCvFrame() if depth_queue.has() else None
                )

                # print(color_frame is not None, mono_frame is not None, depth_frame is not None)

                if color_frame is not None:
                    msg = self.bridge.cv2_to_imgmsg(color_frame, "bgr8")
                    self.rgb_pub.publish(msg)
                if left_frame is not None:
                    msg = self.bridge.cv2_to_imgmsg(left_frame, "mono8")
                    self.left_pub.publish(msg)
                if right_frame is not None:
                    msg = self.bridge.cv2_to_imgmsg(right_frame, "mono8")
                    self.right_pub.publish(msg)
                if depth_frame is not None:
                    msg = self.bridge.cv2_to_imgmsg(depth_frame, "mono16")
                    self.depth_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    cam = LuxonisCamera()
    rclpy.spin(cam)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
