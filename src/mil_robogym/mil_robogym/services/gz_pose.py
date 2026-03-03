from datetime import datetime as dt

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GZNode
from mil_msgs.srv import GetGZPose
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from .types import Pose


class GZPoseService(Node):
    """
    Service that handles reading and modifying poses in GZ Sim.
    """

    def __init__(self, world_name: str = "robosub_2025", model_name: str = "sub9"):

        super().__init__("gz_pose_service")

        self.last_pose: Pose = None

        self.model_name = model_name

        self.gz_node = GZNode()
        self.gz_node.subscribe(
            Pose_V,
            f"/world/{world_name}/pose/info",
            self._update_logged_pose,
        )

        self.get_pose_srv = self.create_service(
            GetGZPose,
            "get_gz_pose",
            self.get_gz_pose,
        )

    def get_gz_pose(self, request, response):
        """
        Find the pose message for the requested model.
        """

        if self.last_pose:
            response.x = self.last_pose["x"]
            response.y = self.last_pose["y"]
            response.z = self.last_pose["z"]
            response.roll = self.last_pose["roll"]
            response.pitch = self.last_pose["pitch"]
            response.yaw = self.last_pose["yaw"]

            return response

        raise ValueError(f"Could not get pose information for model {self.model_name}")

    def _update_logged_pose(self, msg: Pose_V):

        model_pose = next(
            (pose for pose in msg.pose if pose.name == self.model_name),
            None,
        )

        if model_pose:
            roll, pitch, yaw = euler_from_quaternion(
                [
                    model_pose.orientation.x,
                    model_pose.orientation.y,
                    model_pose.orientation.z,
                    model_pose.orientation.w,
                ],
            )

            self.last_pose = Pose(
                model_name=self.model_name,
                last_update=dt.now(),
                x=model_pose.position.x,
                y=model_pose.position.y,
                z=model_pose.position.z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
            )

        else:
            self.last_pose = None


def main():
    rclpy.init()

    gz_pose_service = GZPoseService()

    rclpy.spin(gz_pose_service)

    rclpy.shutdown()
