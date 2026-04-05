import json
import queue
import threading
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image


class DataCollectorService(Node):
    """
    Service that handles storing latest snapshots of topics of interest.
    """

    def __init__(self):

        super().__init__("data_collector_service")

        self.subscribers = {}
        self.latest_data = {}

        self.bridge = CvBridge()

        self.establish_subscriptions_srv = self.create_service(
            EstablishSubscriptions,
            "establish_subscriptions",
            self.establish_subscriptions,
        )

        self.get_snapshot_srv = self.create_service(
            GetSnapshot,
            "get_snapshot",
            self.get_snapshot,
        )

        # Count number of files in folders for abstract data
        self.abstract_data_counters = {}

        # Image data saving attributes
        self.save_queue = queue.Queue()
        self.worker_thread = threading.Thread(
            target=self._image_saver_worker,
            daemon=True,
        )
        self.worker_thread.start()

    def establish_subscriptions(
        self,
        request: EstablishSubscriptions.Request,
        response: EstablishSubscriptions.Response,
    ) -> EstablishSubscriptions.Response | None:
        """
        Create subscribers for each topic provided.
        """

        # Refresh available topics
        self.latest_data = {}
        topic_map = dict(self.get_topic_names_and_types())

        # Find the union between existing subscribers and request and delete those that are not required.
        union_topics = set(request.topics) & set(self.subscribers)
        for topic in self.subscribers:
            if topic not in union_topics:
                del self.subscribers[topic]

        for topic in request.topics:

            # Check if topic exists
            if topic not in topic_map:
                response.failed_topics.append(topic)
                self.get_logger().error(f"Topic '{topic}' not found in ROS graph.")
                continue

            # Check if subscriber already exists
            if topic in self.subscribers:
                response.active_topics.append(topic)
                self.get_logger().info(f"Topic '{topic}' already has a subscriber.")
                continue

            # Get topic type
            type_str = topic_map[topic][0]
            msg_type = get_message(type_str)

            # Create subscriber
            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self._callback(msg, t),
                10,
            )

            response.active_topics.append(topic)
            self.subscribers[topic] = sub
            self.get_logger().info(f"Subscriber for '{topic}' successfully created.")

        # Return response
        response.success = len(response.failed_topics) == 0
        return response

    def get_snapshot(
        self,
        request: GetSnapshot.Request,
        response: GetSnapshot.Response,
    ) -> GetSnapshot.Response | None:
        """
        Return a serialized snapshot of the latest data.
        """
        mapped_data = {}
        demo_path = request.demo_path

        for topic, msg in self.latest_data.items():

            if type(msg) is Image:

                if demo_path:  # Save data to location

                    # Convert ROS image to OpenCV
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                    # Create folder if not already made
                    topic_dir = (
                        Path(demo_path) / "data" / (topic.strip("/").replace("/", "_"))
                    )
                    topic_dir.mkdir(parents=True, exist_ok=True)

                    # Create file_path
                    if topic not in self.abstract_data_counters:
                        try:
                            self.abstract_data_counters[topic] = len(
                                list(topic_dir.iterdir()),
                            )
                        except FileNotFoundError:
                            self.abstract_data_counters[topic] = 0

                    img_index = self.abstract_data_counters[topic]

                    filepath = topic_dir / f"img_{img_index}.jpg"

                    self.abstract_data_counters[topic] += 1

                    # Save image
                    self.save_queue.put((filepath, cv_img))

                    mapped_data[topic] = img_index

                else:
                    # Return the image message in response
                    response.image_topics.append(topic)
                    response.image_data.append(msg)

            else:
                mapped_data[topic] = message_to_ordereddict(msg)

        response.data = json.dumps(mapped_data)

        return response

    def _callback(self, msg, topic) -> None:
        self.latest_data[topic] = msg

    def _image_saver_worker(self) -> None:
        while True:

            filepath, cv_img = self.save_queue.get()

            try:
                cv2.imwrite(str(filepath), cv_img)
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")
            finally:
                self.save_queue.task_done()


def main():
    """
    Main entry point to launch the data collector service.
    """
    rclpy.init()

    data_collector_service = DataCollectorService()

    rclpy.spin(data_collector_service)

    rclpy.shutdown()
