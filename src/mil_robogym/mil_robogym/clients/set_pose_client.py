import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GZNode
from rclpy.node import Node

WORLD_NAME = "robosub_2025"
MODEL_NAME = "sub9"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"


class SetPoseClient(Node):
    """
    ROS2 node that uses gz-transport to send set_pose requests for sub9.
    """

    def __init__(self):
        super().__init__("gz_set_pose_client")
        self.gz_node = GZNode()
        self.get_logger().info(
            f"SetPoseClient initialized. Will publish to: {SET_POSE_SERVICE}",
        )

    def set_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ):
        """
        Send a set_pose request to Gazebo for the configured model.

        Args:
            x, y, z:          Position in metres (world frame)
            roll, pitch, yaw: Orientation in radians (world frame)
        """
        # --- Build the Pose message ---
        pose_msg = Pose()
        pose_msg.name = MODEL_NAME

        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z

        # Convert RPY to quaternion
        import math

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        pose_msg.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # --- Call the gz-transport service ---
        timeout_ms = 2000

        success, result = self.gz_node.request(
            SET_POSE_SERVICE,
            pose_msg,
            Pose,
            Boolean,
            timeout_ms,
        )

        if success and result:
            self.get_logger().info(
                f"Pose set successfully: "
                f"pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                f"rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})",
            )
        else:
            self.get_logger().error(
                f"Failed to set pose (success={success}, result={result}). "
                f"Is Gazebo running with world '{WORLD_NAME}'?",
            )

        return success and result


def main(args=None):
    rclpy.init(args=args)

    client = SetPoseClient()

    # Example: move sub9 to (1, 2, -3) with no rotation
    client.set_pose(x=1.0, y=2.0, z=-3.0)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
