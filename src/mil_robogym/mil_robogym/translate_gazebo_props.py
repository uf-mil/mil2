#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose


class TranslateGzProps(Node):
    def __init__(self):
        super().__init__('translate_gz_props')

        self.world_name = 'default'

        # 👇 List your prop names here
        self.entities = [
            'sub9'
        ]

        self.set_pose_cli = self.create_client(
            SetEntityPose,
            f'/world/{self.world_name}/entity/set_pose'
        )

        while not self.set_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEntityPose service...')

        self.translate_entities()

    def translate_entities(self):
        for name in self.entities:
            pose = Pose()

            # IMPORTANT:
            # SetEntityPose is ABSOLUTE, not relative
            # So you must know or track the current pose
            pose.position.y += 1.0  # +1m right

            set_req = SetEntityPose.Request()
            set_req.name = name
            set_req.pose = pose

            self.set_pose_cli.call_async(set_req)

            self.get_logger().info(f'Moved {name} +1m in Y')

        rclpy.shutdown()


def main():
    rclpy.init()
    TranslateGzProps()


if __name__ == '__main__':
    main()

