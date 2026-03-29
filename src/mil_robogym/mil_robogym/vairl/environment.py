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
        max_vals: np.ndarray = np.zeros(4, dtype=np.float32),
        input_features: list[str] = [],
        random_spawn_space: RandomSpawnSpace | None = None,
        data_collector_client: DataCollectorClient | None = None,
        move_client: MoveClient | None = None,
        set_pose_client: SetPoseClient | None = None,
        controller_client: ControllerClient | None = None,
        localization_client: LocalizationClient | None = None,
        initialized: bool = False,
    ):
        super().__init__()

        self.initialized = initialized

        if initialized:

            # Pool clamping boundaries
            self.x_min, self.x_max = -11.0, 11.0
            self.y_min, self.y_max = -25.0, 25.0
            self.z_min, self.z_max = -1.0, 0.0

            # Data Collector
            self.data_collector_client = data_collector_client
            self.data_collector_client.get_snapshot()  # Initialize service with request

            # Clients
            self.move_client = move_client or MoveClient()
            self.set_pose_client = set_pose_client or SetPoseClient()
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
            self.pose = np.zeros(4, dtype=np.float32)
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
            low=-max_vals,
            high=max_vals,
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
        self.pose = np.array([x, y, z, yaw])
        self.set_pose_client.set_pose(x, y, z, yaw=yaw)

        # Reset controller and localization
        # self.localization_client.reset_localization()
        # self.controller_client.reset_controller()

        # Send zero movement
        # self.move_client.move((0.0, 0.0, 0.0, 0.0))

        # Move to spawn position to ground movement client.
        # self.move_client.move((0, 0, 0, 0))

        # Record state
        self.state = self.data_collector_client.get_flattened_snapshot_values(
            self.input_features,
        )

        if GYMNASIUM:
            return self.state.copy(), {}

        return np.array(self.state.copy())

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
        next_pose, within_bounds = self._apply_action_with_bounds(self.pose, action)

        self.pose = next_pose
        x, y, z, yaw = self.pose

        env_reward = 0.0

        # Fast move
        self.set_pose_client.set_pose(x, y, z, yaw=yaw)

        # Publish a goal pose and wait for action to be completed
        # did_successful_movement = self.move_client.move((dx, dy, dz, dyaw))

        # Determine if it hits an obstacle and deduct points
        if not within_bounds:
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

    def _apply_action_with_bounds(self, pose, action):
        """
        Applies action to pose while enforcing pool bounds.

        Returns:
            new_pose (np.ndarray): bounded pose
            within_bounds (bool): True if no clipping occurred
        """
        action = np.asarray(action, dtype=np.float32)
        pose = np.asarray(pose, dtype=np.float32)

        # Proposed new pose
        proposed = pose + action

        # Clamp to bounds
        clamped = proposed.copy()
        clamped[0] = np.clip(proposed[0], self.x_min, self.x_max)  # x
        clamped[1] = np.clip(proposed[1], self.y_min, self.y_max)  # y
        clamped[2] = np.clip(proposed[2], self.z_min, self.z_max)  # z
        clamped[3] = proposed[3]  # yaw (no bounds unless you want wrap)

        # Check if anything was clipped
        within_bounds = np.allclose(proposed[:3], clamped[:3])

        return clamped, within_bounds
