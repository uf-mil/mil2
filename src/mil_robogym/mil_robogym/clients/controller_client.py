import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool


class ControllerClient(Node):
    """
    Handles starting and stopping PID controls.
    """

    def __init__(self):

        super().__init__("controller_client")

        self.start_client = self.create_client(
            SetBool,
            "/pid_controller/enable",
        )

        self.reset_client = self.create_client(
            Empty,
            "/pid_controller/reset",
        )

    def start_controller(self):
        """
        Sends an empty request to the enable service to start controller.
        """
        self._wait_for_services()

        req = SetBool.Request()
        req.data = True

        future = self.start_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def reset_controller(self):
        """
        Sends an empty request to reset controller.
        """
        self._wait_for_services()

        req = Empty.Request()

        future = self.reset_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def _wait_for_services(self):

        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /subjugator_localization/enable service...",
            )

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /subjugator_localization/reset service...",
            )
