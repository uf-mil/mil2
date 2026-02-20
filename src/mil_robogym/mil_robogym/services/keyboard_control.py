import rclpy
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as LaunchNode
from mil_msgs.srv import ToggleKeyboardControls
from rclpy.node import Node


class KeyboardControlService(Node):
    """
    Service that enables and disables keyboard controls.
    """

    def __init__(self):

        super().__init__("keyboard_control_service")

        self.key_board_control_srv = self.create_service(
            ToggleKeyboardControls,
            "toggle_keyboard_controls",
            self.toggle_keyboard_controls,
        )

        self.launch_description = LaunchDescription(
            [
                LaunchNode(
                    package="subjugator_keyboard_control",
                    executable="subjugator_keyboard_control",
                    name="subjugator_keyboard_control",
                    output="screen",
                ),
            ],
        )

        self.launch_service = LaunchService()

        self.launch_service.include_launch_description(self.launch_description)

    def toggle_keyboard_controls(self, request, response):
        """
        Enable or stop keyboard service upon request.
        """

        if request.enable:
            self.launch_service.run()
        else:
            self.launch_service.shutdown()

        return response


def main():
    rclpy.init()

    keyboard_control_service = KeyboardControlService()

    rclpy.spin(keyboard_control_service)

    rclpy.shutdown()
