import rclpy
from mil_msgs.srv import GetGZPose
from rclpy.node import Node


class ModelPoseClient(Node):
    """
    Client node that gets pose data for the model being queried by the service.
    """

    def __init__(self):

        super().__init__("gz_pose_client")

        self.client = self.create_client(GetGZPose, "get_gz_pose")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("gz_pose_service not available, waiting again...")

        self.request = GetGZPose.Request()

    def send_request(self):

        self.future = self.client.call_async(self.request)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
