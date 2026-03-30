import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class SimActuatorClient(Node):

    def __init__(self):
        super().__init__("sim_actuator_client")
        self._timeout = 1.0

        # Register GripperControl plugin service client and check if it times out
        self.gripper_client = self.create_client(SetBool, "gripper_control/set_open")
        while not self.gripper_client.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().info("Waiting for gripper_control/set_open service...")

        # Register MarbleDropper plugin service client and check if it times out
        # self.marble_dropper_client = self.create_client(SetBool,"marble_dropper/set_open")
        # while not self.marble_dropper_client.wait_for_service(timeout_sec=self._timeout):
        #     self.get_logger().info("Waiting for marble_dropper/set_open service...")

        # Register Torpedo plugin service client and check if it times out
        # self.torpedo_client = self.create_client(SetBool, "torpedo/fire")
        # while not self.torpedo_client.wait_for_service(timeout_sec=self._timeout):
        #     self.get_logger().info("Waiting for torpedo/fire service...")

    # Create request and populate with boolean status, then call the appropriate service client
    def actuate_gripper(self, isOpen: bool):
        request = SetBool.Request()
        request.data = isOpen
        self.gripper_client.call_async(request)

    def actuate_marble_dropper(self, isOpen: bool):
        request = SetBool.Request()
        request.data = isOpen
        self.marble_dropper_client.call_async(request)

    def actuate_torpedo(self, launchStatus: bool):
        request = SetBool.Request()
        request.data = launchStatus
        self.torpedo_client.call_async(request)


def main():
    rclpy.init()
    clientNode = SimActuatorClient()
    clientNode.actuate_gripper(True)
    # clientNode.actuate_marble_dropper(True)
    # clientNode.actuate_torpedo(True)
    rclpy.spin(clientNode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
