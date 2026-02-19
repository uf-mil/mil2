from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GZNode


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

        response = Boolean()

        result, response = self.gz_node.request(
            self.service_name,
            request,
            WorldControl,
            Boolean,
            self.timeout,
        )

        print(result, response)

    def pause_simulation(self):
        """
        Pauses the simulation.
        """
        request = WorldControl()
        request.pause = True

        response = Boolean()

        result, response = self.gz_node.request(
            self.service_name,
            request,
            WorldControl,
            Boolean,
            self.timeout,
        )

        print(result, response)
