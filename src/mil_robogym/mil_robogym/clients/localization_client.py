import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class LocalizationClient(Node):
    """
    Controls localization.
    """

    def __init__(self):

        super().__init__("localization_client")

        self.start_client = self.create_client(
            Empty,
            "/subjugator_localization/enable",
        )

        self.reset_client = self.create_client(
            Empty,
            "/subjugator_localization/reset",
        )

        self._wait_for_service()

    def start_localization(self):
        """
        Start localization.
        """
        req = Empty.Request()

        future = self.start_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def reset_localization(self):
        """
        Start localization.
        """
        req = Empty.Request()

        future = self.reset_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def _wait_for_service(self):

        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /subjugator_localization/enable service...",
            )

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /subjugator_localization/reset service...",
            )
