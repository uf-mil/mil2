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
        self.device = dai.Device(self.__pipeline)

        self.cam_rgb = self.__pipeline.create(dai.node.ColorCamera)
        self.cam_mono = self.__pipeline.create(dai.node.MonoCamera)
        self.stereo = self.__pipeline.create(dai.node.StereoDepth)

        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam_mono.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        self.stereo.setLeftmonoCheck(True)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        self.stereo.setSubpixel(True)
        self.stereo.setInputResolution(1280, 720)

        self.cam_mono.out.link(self.stereo.mono)

        self.xout_color = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_color.setStreamName("cam_rgb")
        self.cam_rgb.video.link(self.xout_color.input)

        self.xout_mono = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_mono.setStreamName("cam_mono")
        self.cam_mono.video.link(self.xout_mono.input)

        self.xout_depth = self.__pipeline.create(dai.node.XLinkOut)
        self.xout_depth.setStreamName("cam_depth")
        self.stereo.depth.link(self.xout_depth.input)

        self.bridge = CvBridge()

    def run(self):
        color_queue = self.device.getOutputQueue(
            name="cam_rgb",
            maxSize=4,
            blocking=False,
        )
        mono_queue = self.device.getOutputQueue(
            name="cam_mono",
            maxSize=4,
            blocking=False,
        )
        depth_queue = self.device.getOutputQueue(
            name="cam_depth",
            maxSize=4,
            blocking=False,
        )

        color_frame = color_queue.get().getCvFrame() if color_queue.has() else None
        mono_frame = mono_queue.get().getCvFrame() if mono_queue.has() else None
        depth_frame = depth_queue.get().getCvFrame() if depth_queue.has() else None

        color_image = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
        mono_image = self.bridge.cv2_to_imgmsg(mono_frame, encoding="bgr8")
        depth_image = self.bridge.cv2_to_imgmsg(depth_frame, encoding="bgr8")

        self.rgb_pub.publish(color_image)
        self.mono_pub.publish(mono_image)
        self.depth_pub.publish(depth_image)


def main():
    rclpy.init()
    LuxonisCamera().run()


if __name__ == "__main__":
    main()
