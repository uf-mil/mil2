import json
import queue
import threading
from contextlib import suppress
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Empty


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

        self.reset_image_counters_srv = self.create_service(
            Empty,
            "reset_image_counters",
            self._reset_image_counters,
        )

        # Count number of files in folders for image data
        self.image_data_counters = {}

        # Image data saving attributes
        self.image_fields_map = {}
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
        for topic in list(self.subscribers):
            if topic not in union_topics:
                subscription = self.subscribers.pop(topic)
                with suppress(Exception):
                    self.destroy_subscription(subscription)

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

        # Set up image folder paths if any
        self.image_data_counters = {}
        for image_field in request.image_fields:
            try:
                # Split into topic and optional field path
                if ":" in image_field:
                    topic, field_str = image_field.split(":", 1)
                    field_path = field_str.split(".")
                else:
                    topic = image_field
                    field_path = []  # whole message is the image

                # Validate topic exists
                if topic not in topic_map:
                    self.get_logger().error(f"Image field topic '{topic}' not found.")
                    continue

                # Create directory name (safe for filesystem)
                img_dir = (
                    image_field.strip("/")
                    .replace("/", "_")
                    .replace(":", "_")
                    .replace(".", "_")
                )

                # Store mapping
                self.image_fields_map[topic] = {
                    "field_path": field_path,
                    "dir": img_dir,
                }

                self.get_logger().info(
                    f"Registered image field '{image_field}' -> {img_dir}",
                )

            except Exception as e:
                self.get_logger().error(
                    f"Failed to process image field '{image_field}': {e}",
                )

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

            # NOTE: This assumes only one nested image msg per topic
            if topic in self.image_fields_map:

                fields = self.image_fields_map[topic]["field_path"]
                img_msg = self._extract_nested_field(msg, fields)

                if demo_path:  # Save data to location

                    save_dir = self.image_fields_map[topic]["dir"]

                    # Convert ROS image to OpenCV
                    cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

                    # Create folder if not already made
                    topic_dir = Path(demo_path) / "data" / save_dir
                    topic_dir.mkdir(parents=True, exist_ok=True)

                    # Create file_path
                    image_data_counter_key = f"{topic}_{'_'.join(fields)}"
                    if image_data_counter_key not in self.image_data_counters:
                        try:
                            self.image_data_counters[image_data_counter_key] = len(
                                list(topic_dir.iterdir()),
                            )
                        except FileNotFoundError:
                            self.image_data_counters[image_data_counter_key] = 0

                    img_index = self.image_data_counters[image_data_counter_key]

                    filepath = topic_dir / f"img_{img_index}.jpg"

                    self.image_data_counters[image_data_counter_key] += 1

                    # Save image
                    self.save_queue.put((filepath, cv_img))

                else:
                    # Return the image message in response
                    response.image_topics.append(topic)
                    response.image_data.append(img_msg)

            else:
                # NOTE: Unordered set data can be serialized.
                # NOTE: Currently if a message has a nested image it will be treated as an image topic
                mapped_data[topic] = message_to_ordereddict(msg)

        response.data = json.dumps(mapped_data)

        return response

    def _reset_image_counters(
        self,
        request: Empty.Request,
        response: Empty.Response,
    ) -> Empty.Response | None:
        self.image_data_counters = {}
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

    def _extract_nested_field(self, msg, field_path):
        """
        Traverse a message using a field path like ['a', 'b', 'c'].
        """
        current = msg
        for field in field_path:
            current = getattr(current, field, None)
            if current is None:
                return None
        return current


def main():
    """
    Main entry point to launch the data collector service.
    """
    rclpy.init()

    data_collector_service = DataCollectorService()

    rclpy.spin(data_collector_service)

    rclpy.shutdown()
