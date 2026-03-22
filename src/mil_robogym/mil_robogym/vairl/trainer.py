from mil_robogym.data_collection.types import RoboGymProjectYaml

from ..client.data_collector_client import DataCollectorClient
from ..clients.controller_client import ControllerClient
from ..clients.localization_client import LocalizationClient
from ..clients.world_control_client import WorldControlClient
from .environment import Environment


class Trainer:
    """
    Creates environment and configures VAIRL with all necessary parameters for training.
    """

    def __init__(
        self,
        project: RoboGymProjectYaml,
        max_step_count: int = 40,
        max_step_size: int = 0.5,
        num_episodes: int = 500,
        rollout_steps: int = 2048,
        generator_learning_rate: float = 1e-3,
        discriminator_learning_rate: float = 3e-3,
        z_size: int = 6,
        e_hidden_size: int = 128,
        i_c: int = 0.5,
        beta_step_size: float = 1e-3,
        gamma: float = 0.99,
        save_every: int = 10,
        seed: int = 42,
        env_id: str = "VairlROS2-v0",
        expert_noise_std: float = 1e-4,
    ):
        # Save all parameters to class attributes
        self.max_step_count = max_step_count
        self.max_step_size = max_step_size
        self.num_episodes = num_episodes
        self.rollout_steps = rollout_steps
        self.generator_learning_rate = generator_learning_rate
        self.discriminator_learning_rate = discriminator_learning_rate
        self.z_size = z_size
        self.e_hidden_size = e_hidden_size
        self.i_c = i_c
        self.beta_step_size = beta_step_size
        self.gamma = gamma
        self.save_every = save_every
        self.seed = seed
        self.env_id = env_id
        self.expert_noise_std = expert_noise_std

        # ROS2 components
        self.world_control_client = WorldControlClient()
        self.controller_client = ControllerClient()
        self.localization_client = LocalizationClient()

        # Start up data collector client
        self.data_collector_client = DataCollectorClient()
        self.data_collector_client.establish_subscriptions(
            list(project["input_topics"].keys()),
        )

        # Training components
        self.environment = Environment(
            seed,
            max_step_count,
            project["tensor_spec"]["input_features"],
            project["random_spawn_space"],
            self.data_collector,
            self.controller_client,
            self.localization_client,
        )

    def train(self):
        """
        Train the VAIRL algorithm.
        """

        train_discriminator_flag = True

        # TODO: Configure metric outputs

        # TODO: Register environment

        # TODO: Load in expert demonstrations

        # TODO: Create vec env

        # TODO: Establish TRPO policy

        # TODO: Instantiate Reward network

        # TODO: Instantiate VAIRL

        # TODO: Create VAIRL environment

        for episode in range(NUM_EPISODES):

            # TODO: Training loop

            pass

    def _ready_simulation(self):
        """
        Starts up simulation with controls and localization.
        """
        self.world_control_client.play_simulation()
        self.localization_client.start_localization()
        self.controller_client.start_controller()
