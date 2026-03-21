import rclpy
from mil_msgs.srv import EstablishSubscriptions, GetSnapshot
from rclpy.node import Node


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

    def get_snapshot(self):

        req = GetSnapshot.Request()

        future = self.snapshot_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()
