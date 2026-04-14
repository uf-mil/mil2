import rclpy
from mil_msgs.msg import TrackedDetection
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class SharkDetect(Node):
    """
    Simple node that parses the yolo detections from sim and tracks detections of shark.
    """

    def __init__(self):

        super().__init__("shark_detect")

        self.seen = False

        self.sub = self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self._callback,
            10,
        )

        self.pub = self.create_publisher(
            TrackedDetection,
            "/shark_tracking",
            10,
        )

    def _callback(self, msg: DetectionArray) -> None:

        pub_msg = TrackedDetection()

        for detection in msg.detections:

            if detection.class_id == 0:

                self.seen = True
                pub_msg.center_x = msg.bbox.center.position.x
                pub_msg.center_y = msg.bbox.center.position.y
                pub_msg.width = msg.bbox.size.x
                pub_msg.height = msg.bbox.size.y
                pub_msg.area = pub_msg.width * pub_msg.height
                pub_msg.seen = self.seen

        pub_msg.center_x = 0.0
        pub_msg.center_y = 0.0
        pub_msg.width = 0.0
        pub_msg.height = 0.0
        pub_msg.area = 0.0
        pub_msg.seen = self.seen

        self.pub.publish(pub_msg)


def main(args=None):

    rclpy.init(args=args)

    shark_detect = SharkDetect()

    rclpy.spin(shark_detect)
