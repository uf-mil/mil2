import rclpy
from mil_msgs.srv import ToggleKeyboardControls


class KeyboardControlClient:
    """
    Client that enables and disables keyboard controls.
    """

    def __init__(self):

        self.request = ToggleKeyboardControls.Request()

    def toggle_keyboard_controls(self, enable: bool):
        """
        Create a one-shot client and send request to toggle keyboard controls.
        """
        rclpy.init()

        node = rclpy.create_node("toggle_keyboard_client")
        client = node.create_client(ToggleKeyboardControls, "toggle_keyboard_controls")

        client.wait_for_service()

        self.request.enable = enable

        future = client.call_async(self.request)
        rclpy.spin_until_future_complete(node, future)

        node.destroy_node()
        rclpy.shutdown()
