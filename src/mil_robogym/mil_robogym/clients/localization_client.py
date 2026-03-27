import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from robot_localization.srv import SetPose
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
            SetPose,
            "subjugator_localization/set_pose",
        )

        # Set up reset request
        self.reset_request = SetPose.Request()
        self.reset_request.pose = PoseWithCovarianceStamped()

        self.reset_request.pose.header.frame_id = "odom"

        self.reset_request.pose.pose.pose.position.x = 0.0
        self.reset_request.pose.pose.pose.position.y = 0.0
        self.reset_request.pose.pose.pose.position.z = 0.0

        self.reset_request.pose.pose.pose.orientation.x = 0.0
        self.reset_request.pose.pose.pose.orientation.y = 0.0
        self.reset_request.pose.pose.pose.orientation.z = 0.0
        self.reset_request.pose.pose.pose.orientation.w = 1.0

        self.reset_request.pose.pose.covariance = [0.0] * 36

    def start_localization(self):
        """
        Start localization.
        """
        self._wait_for_service()

        req = Empty.Request()

        future = self.start_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def reset_localization(self):
        """
        Start localization.
        """
        self._wait_for_service()

        future = self.reset_client.call_async(self.reset_request)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def _wait_for_service(self):

        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /subjugator_localization/enable service...",
            )

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /subjugator_localization/set_pose service...",
            )
