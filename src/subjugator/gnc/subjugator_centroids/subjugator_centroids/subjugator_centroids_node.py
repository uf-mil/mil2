import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from yolo_msgs.msg import Detection, DetectionArray


class SubjugatorCentroidsNode(Node):
    def __init__(self):
        super().__init__("subjugator_centroids_node")

        self.topics: dict[int, Publisher] = {}
        self._tracking_sub = self.create_subscription(
            DetectionArray,
            "yolo/tracking",
            self.tracking_cb,
            10,
        )

    def tracking_cb(self, msg: DetectionArray):
        for detection in msg.detections:
            detection: Detection
            class_id: int = detection.class_id
            class_name: str = detection.class_name
            # id: int = detection.id
            # center_x: float = detection.bbox.center.position.x
            # center_y: float = detection.bbox.center.position.y
            # size_x: float = detection.bbox.size.x
            # size_y: float = detection.bbox.size.y

            # check to see if topic already exists, if it doesn't, create it
            pub_already_exists: bool = class_id in self.topics
            if not pub_already_exists:
                topic_name: str = "centroids/" + class_name
                topic_name = topic_name.replace("-", "_")
                self.topics[class_id] = self.create_publisher(Detection, topic_name, 10)

            self.topics[class_id].publish(detection)

            # print("------------")
            # print(detection.class_id)
            # print(detection.class_name)
            # print(detection.id)
            # print(detection.bbox.center.position.x)
            # print(detection.bbox.center.position.y)
            # print(detection.bbox.size.x)
            # print(detection.bbox.size.y)
            # print("------------")


def main():
    rclpy.init()
    node = SubjugatorCentroidsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
