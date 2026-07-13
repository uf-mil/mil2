import importlib

import rclpy
from rclpy.node import Node

import admission as adm
from subjugator_msgs.srv import Admission
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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
        response.success = False
        try:
            adm.run(importlib.import_module(msg.name).main())
            response.success = True
        except Exception as e:
            print(e)
        return response

if __name__ == "__main__":
    adm.should_shutdown = False
    rclpy.spin(AdmissionSrvNode())
    rclpy.shutdown()
