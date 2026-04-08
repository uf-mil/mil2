import numpy as np

from mil_robogym.data_collection.load_saved_agent import (
    LoadedAgent,
)
from mil_robogym.data_collection.types import Coord4D, RoboGymProjectYaml

from ..clients.controller_client import ControllerClient
from ..clients.data_collector_client import DataCollectorClient
from ..clients.get_pose_client import GetPoseClient
from ..clients.localization_client import LocalizationClient
from ..clients.move_client import MoveClient
from ..clients.set_pose_client import SetPoseClient
from ..clients.world_control_client import WorldControlClient


class Tester:
    """
    Class responsible for loading in an agent and running the testing loop.
    """

    def __init__(self, project: RoboGymProjectYaml):

        # Create clients
        self.data_collector_client = DataCollectorClient()
        self.data_collector_client.establish_subscriptions(project)

        self.controller_client = ControllerClient()
        self.localization_client = LocalizationClient()
        self.move_client = MoveClient()
        self.set_pose_client = SetPoseClient()
        self.get_pose_client = GetPoseClient()
        self.world_control_client = WorldControlClient()

        # Agent parameters
        self.agent = None
        self.max_step_count = None

        random_spawn_space = project["random_spawn_space"]
        c1 = random_spawn_space["coord1_4d"]
        c2 = random_spawn_space["coord2_4d"]
        self.min_coord = np.minimum(c1, c2)
        self.max_coord = np.maximum(c1, c2)

        self.input_features = project["tensor_spec"]["input_features"]

    def set_agent(self, agent: LoadedAgent) -> None:
        """
        Set the selected agent to be tested.
        """
        self.agent = agent
        self.max_step_count = agent.handle.training_settings["max_step_count"]

    def test_agent(self) -> None:
        """
        Run one path with the agent.
        """
        if self.agent:

            # Place the sub in a random position
            x, y, z, yaw = np.random.uniform(low=self.min_coord, high=self.max_coord)
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

            # Ready simulation
            self.world_control_client.play_simulation()

            self.localization_client.start_localization()
            self.controller_client.start_controller()

            self.localization_client.reset_localization()
            self.controller_client.reset_controller()

            # Iterate and generate movement
            for i in range(self.max_step_count):

                # Get state
                state = self.data_collector_client.get_flattened_snapshot_values(
                    self.input_features,
                )

                # Generate action
                action = self.agent.predict(state)

                # Transform to sub reference frame
                move_coord = self._world_to_body(action)

                # Move sub
                self.move_client.move(move_coord)

            # Stop simulation
            self.world_control_client.pause_simulation()

            # Notify that the test has finished
            self.move_client.get_logger().info("Testing finished!")

        else:
            raise ValueError(
                "No agent has been selected. Select an agent and try again.",
            )

    def _world_to_body(self, move_coord: Coord4D) -> Coord4D:
        sub_pose = self.get_pose_client.send_request()
        sub_yaw = sub_pose.yaw

        self.move_client.get_logger().info(
            f"World frame vector {move_coord} | Sub yaw: {sub_yaw}",
        )

        dx_w, dy_w, z, yaw_w = move_coord

        dx_b = np.cos(sub_yaw) * dx_w + np.sin(sub_yaw) * dy_w
        dy_b = -np.sin(sub_yaw) * dx_w + np.cos(sub_yaw) * dy_w

        return (dx_b, dy_b, z, yaw_w)
