import json

import rclpy
from cv_bridge import CvBridge
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node
from std_srvs.srv import Empty

from mil_robogym.data_collection.types import (
    NonNumericTopicFieldSelection,
    RoboGymProjectYaml,
)
from mil_robogym.data_collection.utils import flatten_value


class DataCollectorClient(Node):
    """
    Client that sends requests to the DataCollectorService.
    """

    def __init__(self):
        super().__init__("data_collector_client")

        self.bridge = CvBridge()

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

    def get_flattened_snapshot_values(self, project: RoboGymProjectYaml) -> list[any]:

        input_features = project["tensor_spec"]["input_features"]
        non_numeric_features = project["input_non_numeric_topics"]

        # Compose full list of input features
        input_features.extend(
            self._get_abstract_data_feature_list(non_numeric_features),
        )

        snapshot = self.get_snapshot()

        data = json.loads(snapshot.data)
        img_msgs = zip(snapshot.image_topics, snapshot.image_data)

        flattened_data = []

        if data or img_msgs:
            # Extract data
            filtered_data = self._flatten_and_filter_state_fields(
                data,
                img_msgs,
                input_features,
            )

            if len(filtered_data) != len(input_features):
                return []

            flattened_data = [filtered_data[key] for key in input_features]

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

                    data_path = topic + (
                        f":{field_path}" if field_path != "data" else ""
                    )

                    image_data_paths.append(data_path)

        return image_data_paths

    def _get_abstract_data_feature_list(
        self,
        topics: dict[str, list[NonNumericTopicFieldSelection]],
    ) -> list[str]:
        data_paths = []

        for topic, data_list in topics.items():

            for data in data_list:

                if data["data_type"] == "image":
                    data_paths.append(topic)

                else:
                    data_paths.append(f"{topic}:{data['field_path']}")

        return data_paths

    # TODO: This is inefficient, implement a fast mapper on the server side.
    def _flatten_and_filter_state_fields(
        self,
        data: dict,
        img_msgs: zip,  # image_topic, image_msg
        input_features: list[str],
    ) -> dict:
        """
        Flatten dict into column names and keep only desired column names.
        """
        flattened_states = {}

        for topic, msg in data.items():

            temp = {}
            flatten_value(msg, "", temp)

            # Iterate through numeric and set data
            for key, value in temp.items():

                feature_name = f"{topic}:{key}"
                flattened_states[feature_name] = value

            # Iterate through image data
            for topic, img_msg in img_msgs:
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
                flattened_states[topic] = cv_img

        features_allowed = set(input_features)

        # Compose dict for features
        return {k: v for k, v in flattened_states.items() if k in features_allowed}
