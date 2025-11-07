#!/usr/bin/env python3
import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import SetParameters
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


class SwapYoloModel(Node):
    def __init__(self):
        super().__init__("swap_yolo_model")

        # Parameters you can set before calling the service
        self.declare_parameter("node_name", "/yolo/yolo_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("threshold", Parameter.Type.DOUBLE)
        self.declare_parameter("device", Parameter.Type.STRING)
        self.declare_parameter("half", Parameter.Type.BOOL)

        self.srv = self.create_service(Trigger, "swap_yolo_model", self.on_trigger)

    # helper to wait for service call
    def _wait(self, client, req, timeout=4.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    # helper to read optional parameters (unset -> None)
    def _get_optional(self, name: str):
        try:
            # If declared but unset, its type_ will be NOT_SET
            p = self.get_parameter(name)
            return None if p.type_ is Parameter.Type.NOT_SET else p.value
        except ParameterUninitializedException:
            # Not declared at all => treat as None
            return None

    def on_trigger(self, _req, res):
        target = self.get_parameter("node_name").value
        model = self.get_parameter("model_path").value
        thr = self._get_optional("threshold")
        dev = self._get_optional("device")
        half = self._get_optional("half")

        if not model:
            res.success = False
            res.message = "Set parameter 'model_path' before calling."
            return res

        # Clients
        get_state = self.create_client(GetState, f"{target}/get_state")
        change = self.create_client(ChangeState, f"{target}/change_state")
        setparam = self.create_client(SetParameters, f"{target}/set_parameters")

        for c in (get_state, change, setparam):
            if not c.wait_for_service(timeout_sec=2.0):
                res.success = False
                res.message = f"Service not available: {c.srv_name}"
                return res

        # Deactivate if active
        gs = self._wait(get_state, GetState.Request(), timeout=3.0)
        if gs and gs.current_state.id == 3:
            req_deact = ChangeState.Request()
            req_deact.transition.id = Transition.TRANSITION_DEACTIVATE
            if self._wait(change, req_deact, timeout=5.0) is None:
                res.success = False
                res.message = "Deactivate failed"
                return res

        # Set parameters on yolo_node
        params = [Parameter("model", Parameter.Type.STRING, model)]
        if thr is not None:
            params.append(Parameter("threshold", Parameter.Type.DOUBLE, float(thr)))
        if dev is not None:
            params.append(Parameter("device", Parameter.Type.STRING, str(dev)))
        if half is not None:
            params.append(Parameter("half", Parameter.Type.BOOL, bool(half)))

        spreq = SetParameters.Request()
        spreq.parameters = [p.to_parameter_msg() for p in params]
        if self._wait(setparam, spreq, timeout=90.0) is None:
            res.success = False
            res.message = "SetParameters failed"
            return res

        # Activate again
        req_act = ChangeState.Request()
        req_act.transition.id = Transition.TRANSITION_ACTIVATE
        if self._wait(change, req_act, timeout=5.0) is None:
            res.success = False
            res.message = "Activate failed"
            return res

        res.success = True
        res.message = f"Swapped to model: {model}"
        return res


def main():
    rclpy.init()
    rclpy.spin(SwapYoloModel())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
