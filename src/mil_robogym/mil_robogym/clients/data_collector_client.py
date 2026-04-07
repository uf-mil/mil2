import json

import rclpy
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node
from std_srvs.srv import Empty

from mil_robogym.data_collection.types import RoboGymProjectYaml
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

        self.reset_image_counters_client = self.create_client(
            Empty,
            "reset_image_counters",
        )

        self._wait_for_services()

    def _wait_for_services(self):
        while not self.establish_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for establish_subscriptions service...")

        while not self.snapshot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_snapshot service...")

    def establish_subscriptions(self, project: RoboGymProjectYaml):

        req = EstablishSubscriptions.Request()
        req.topics = list(project["input_topics"].keys())
        req.image_fields = self._extract_image_data_paths_from_project(project)

        future = self.establish_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def get_snapshot(self, demo_path: str | None = None):

        req = GetSnapshot.Request()
        req.demo_path = demo_path or ""

        future = self.snapshot_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def reset_image_counters(self):

        req = Empty.Request()

        future = self.reset_image_counters_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def get_flattened_snapshot_values(self, input_features: list[str]) -> list[any]:

        snapshot = self.get_snapshot()

        data = json.loads(snapshot.data)

        images = []
        flattened_data = []

        if data:
            filtered_data = self._flatten_and_filter_state_fields(data, input_features)
            flattened_data = [
                filtered_data[key] for key in input_features
            ]  # Maintain order

        return flattened_data

    def _extract_image_data_paths_from_project(
        self,
        project: RoboGymProjectYaml,
    ) -> list[str]:
        """
        Outputs a string of data paths in the form: /topic:parent_field.child_field OR /topic
        """
        input_non_numeric_topics = project.get("input_non_numeric_topics", {})

        image_data_paths = []

        for topic, data_list in input_non_numeric_topics.items():

            for data in data_list:
                
                if data["data_type"] == "image":

                    field_path = data["field_path"]

                    data_path = topic + (f":{field_path}" if field_path != "data" else "")

                    image_data_paths.append(data_path)

        return image_data_paths

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
