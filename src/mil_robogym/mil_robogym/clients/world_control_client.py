import threading

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GZNode

_WORLD_CONTROL_LOCK = threading.Lock()
_SIMULATION_HOLD_COUNT = 0


class WorldControlClient:
    """
    Client that interfaces with the world control service from gz sim.
    """

    def __init__(self, service_name: str = "/world/robosub_2025/control"):

        self.gz_node = GZNode()

        self.service_name = service_name

        self.timeout = 5000

    def play_simulation(self):
        """
        Enables physics and timer of the gz sim environment.
        """
        request = WorldControl()
        request.pause = False

        self.gz_node.request(
            self.service_name,
            request,
            WorldControl,
            Boolean,
            self.timeout,
        )

    def pause_simulation(self):
        """
        Pauses the simulation.
        """
        request = WorldControl()
        request.pause = True

        self.gz_node.request(
            self.service_name,
            request,
            WorldControl,
            Boolean,
            self.timeout,
        )

    def acquire_simulation_hold(self) -> None:
        """
        Keep simulation running until a matching release call occurs.
        """
        global _SIMULATION_HOLD_COUNT
        with _WORLD_CONTROL_LOCK:
            if _SIMULATION_HOLD_COUNT == 0:
                self.play_simulation()
            _SIMULATION_HOLD_COUNT += 1

    def release_simulation_hold(self) -> None:
        """
        Release a simulation hold and pause only when no holds remain.
        """
        global _SIMULATION_HOLD_COUNT
        with _WORLD_CONTROL_LOCK:
            if _SIMULATION_HOLD_COUNT == 0:
                return
            _SIMULATION_HOLD_COUNT -= 1
            if _SIMULATION_HOLD_COUNT == 0:
                self.pause_simulation()
