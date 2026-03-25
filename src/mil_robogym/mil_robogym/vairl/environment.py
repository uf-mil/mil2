"""
IMPORTANT: THIS ENVIRONMENT IS CONFIGURED FOR MOVEMENT ONLY I.E. 4D. It is not configured for other outputs yet.
"""

import numpy as np

try:
    import gymnasium as gym

    GYMNASIUM = True
except ImportError:  # fallback for older gym
    import gym

    GYMNASIUM = False

from mil_robogym.data_collection.types import RandomSpawnSpace

from ..clients.controller_client import ControllerClient
from ..clients.data_collector_client import DataCollectorClient
from ..clients.localization_client import LocalizationClient
from ..clients.move_client import MoveClient
from ..clients.set_pose_client import SetPoseClient


class Environment(gym.Env):
    """
    Training environment for VAIRL algorithm with Gazebo.
    """

    def __init__(
        self,
        seed: int | None = None,
        max_step_count: int = 40,
        input_features: list[str] = [],
        random_spawn_space: RandomSpawnSpace | None = None,
        data_collector_client: DataCollectorClient | None = None,
        controller_client: ControllerClient | None = None,
        localization_client: LocalizationClient | None = None,
        initialized: bool = False,
    ):
        super().__init__()

        self.initialized = initialized

        if initialized:

            # Data Collector
            self.data_collector_client = data_collector_client
            self.data_collector_client.get_snapshot()  # Initialize service with request

            # Clients
            self.move_client = MoveClient()
            self.set_pose_client = SetPoseClient()
            self.controller_client = controller_client or ControllerClient()
            self.localization_client = localization_client or LocalizationClient()

            # Saved parameters
            self.seed = seed
            self.max_step_count = max_step_count
            self.input_features = input_features

            # Configure random spawn space
            c1 = random_spawn_space["coord1_4d"]
            c2 = random_spawn_space["coord2_4d"]
            self.min_coord = np.minimum(c1, c2)
            self.max_coord = np.maximum(c1, c2)

            # Tracked values
            self.state = None
            self.t = 0
            self.rng = np.random.default_rng(seed)

        # Configure observation space
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(len(input_features),),
            dtype=np.float32,
        )

        # TODO: adapt shape size when we start considering other output like shooting a torpedo.
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32,
        )

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        """
        Place sub in random position based on the random spawn space of the project.
        """
        if not self.initialized:
            raise ValueError(
                "Environment not initialized, will not proceed to train...",
            )

        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self.t = 0

        # Set position
        x, y, z, yaw = np.random.uniform(low=self.min_coord, high=self.max_coord)
        self.set_pose_client.set_pose(x, y, z, yaw=yaw)

        # Reset controller and localization
        self.controller_client.reset_controller()
        self.localization_client.reset_localization()

        # Record state
        self.state = self.data_collector_client.get_flattened_snapshot_values(
            self.input_features,
        )

        if GYMNASIUM:
            return self.state.copy(), {}

        return self.state.copy()

    def step(self, action):
        """
        Move sub by sending a goal pose.
        """
        if not self.initialized:
            raise ValueError(
                "Environment not initialized, will not proceed to train...",
            )

        self.t += 1

        action = np.asarray(action, dtype=np.float32)
        dx, dy, dz, dyaw = action

        env_reward = 0.0

        # Publish a goal pose and wait for action to be completed
        movement = self.move_client.move((dx, dy, dz, dyaw))

        # Determine if it hits an obstacle and deduct points
        if not movement.success:
            env_reward -= 1.0

        # Record next state
        next_state = self.data_collector_client.get_flattened_snapshot_values(
            self.input_features,
        )
        while not next_state:
            self.data_collector.get_logger().warn(
                "Unable to retrieve state, trying again...",
            )
            next_state = self.data_collector_client.get_flattened_snapshot_values(
                self.input_features,
            )

        self.state = next_state

        # TODO: Maybe determine condition for termination based on where most demos terminate
        terminated = False
        truncated = self.t >= self.max_step_count

        if GYMNASIUM:
            return (
                next_state.copy(),
                float(env_reward),
                bool(terminated),
                bool(truncated),
                {},
            )

        done = bool(terminated or truncated)
        return next_state.copy(), float(env_reward), done, {}
