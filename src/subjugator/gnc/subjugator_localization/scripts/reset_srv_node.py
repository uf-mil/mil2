#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from robot_localization.srv import SetPose
from std_srvs.srv import Empty


class ResetLocalizationService(Node):

    def __init__(self):
        super().__init__("reset_localization_service")
        self.srv = self.create_service(
            Empty,
            "subjugator_localization/reset",
            self.reset_localization_callback,
        )
        self.set_pose_client = self.create_client(
            SetPose,
            "subjugator_localization/set_pose",
        )
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def reset_localization_callback(self, request, response):
        self.get_logger().info("Incoming request to reset localization")
        set_pose_request = SetPose.Request()
        set_pose_request.pose = PoseWithCovarianceStamped()
        set_pose_request.pose.header.frame_id = "odom"
        set_pose_request.pose.pose.pose.position.x = 0.0
        set_pose_request.pose.pose.pose.position.y = 0.0
        set_pose_request.pose.pose.pose.position.z = 0.0
        set_pose_request.pose.pose.pose.orientation.x = 0.0
        set_pose_request.pose.pose.pose.orientation.y = 0.0
        set_pose_request.pose.pose.pose.orientation.z = 0.0
        set_pose_request.pose.pose.pose.orientation.w = 1.0
        set_pose_request.pose.pose.covariance = [0.0] * 36
        self.set_pose_client.call_async(set_pose_request)
        response = Empty.Response()
        return response


def main():
    rclpy.init()
    reset_service = ResetLocalizationService()
    rclpy.spin(reset_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
