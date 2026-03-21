import json

import rclpy
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


class DataCollectorService(Node):
    """
    Service that handles storing latest snapshots of topics of interest.
    """

    def __init__(self):

        super().__init__("data_collector_service")

        self.subscribers = {}
        self.latest_data = {}

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

    def establish_subscriptions(
        self,
        request: EstablishSubscriptions.Request,
        response: EstablishSubscriptions.Response,
    ) -> EstablishSubscriptions.Response | None:
        """
        Create subscribers for each topic provided.
        """

        # Refresh available topics
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
        response.data = json.dumps(self.latest_data)
        return response

    def _callback(self, msg, topic) -> None:
        self.latest_data[topic] = message_to_ordereddict(msg)


def main():
    """
    Main entry point to launch the data collector service.
    """
    rclpy.init()

    data_collector_service = DataCollectorService()

    rclpy.spin(data_collector_service)

    rclpy.shutdown()
