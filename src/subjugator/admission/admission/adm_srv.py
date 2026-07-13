import importlib

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from admission import adm
from subjugator_msgs.srv import Admission

class AdmissionSrvNode(Node):
    def __init__(self):
        super().__init__("admission_srv")
        self.srv = self.create_service(
            Admission,
            "/admission",
            self.srv_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def srv_cb(self, msg, response):
        print("running", msg)
        response.success = False
        try:
            adm.run(importlib.import_module(f"admission.{msg.name}").main())
            response.success = True
        except Exception as e:
            print(e)
        print("ran", msg, response.success)
        return response

def main():
    adm.should_shutdown = False
    rclpy.spin(AdmissionSrvNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
