import json

import rclpy
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node

from mil_robogym.data_collection.utils import flatten_value


class DataCollectorClient(Node):
    """
    Client that sends requests to the DataCollectorService.
    """

    def __init__(self):
        super().__init__("data_collector_client")

        self.establish_client = self.create_client(
            EstablishSubscriptions,
            "establish_subscriptions",
        )

        self.snapshot_client = self.create_client(GetSnapshot, "get_snapshot")

        self._wait_for_services()

    def _wait_for_services(self):
        while not self.establish_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for establish_subscriptions service...")

        while not self.snapshot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_snapshot service...")

    def establish_subscriptions(self, topics: list[str]):

        req = EstablishSubscriptions.Request()
        req.topics = topics

        future = self.establish_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def ensure_subscriptions(
        self,
        topics: list[str],
        *,
        operation: str = "establish data collector subscriptions",
    ) -> EstablishSubscriptions.Response:
        response = self.establish_subscriptions(topics)
        if response is None:
            raise RuntimeError(
                f"Failed to {operation}: data collector service returned no response.",
            )

        failed_topics = list(response.failed_topics)
        if failed_topics:
            raise RuntimeError(
                f"Failed to {operation}: topics not found in the ROS 2 graph: {failed_topics}",
            )

        return response

    def get_snapshot(self):

        req = GetSnapshot.Request()

        future = self.snapshot_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def get_flattened_snapshot_values(self, input_features: list[str]) -> list[any]:

        data = json.loads(self.get_snapshot().data)

        if data:
            filtered_data = self._flatten_and_filter_state_fields(data, input_features)
            return [filtered_data[key] for key in input_features]  # Maintain order

        return []

    # TODO: This is inefficient, implement a fast mapper on the server side.
    def _flatten_and_filter_state_fields(
        self,
        data: dict,
        input_features: list[str],
    ) -> dict:
        """
        Flatten dict into column names and keep only desired column names.
        """
        flattened_states = {}

        for topic, msg in data.items():

            temp = {}
            flatten_value(msg, "", temp)

            for key, value in temp.items():

                feature_name = f"{topic}:{key}"
                flattened_states[feature_name] = value

        features_allowed = set(input_features)

        return {k: v for k, v in flattened_states.items() if k in features_allowed}
