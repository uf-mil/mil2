import os

import numpy as np
import torch
from imitation.data import rollout
from imitation.util.networks import RunningNorm
from imitation.util.util import make_vec_env
from sb3_contrib import TRPO

from mil_robogym.data_collection.types import RoboGymProjectYaml

from ..clients.controller_client import ControllerClient
from ..clients.data_collector_client import DataCollectorClient
from ..clients.localization_client import LocalizationClient
from ..clients.move_client import MoveClient
from ..clients.set_pose_client import SetPoseClient
from ..clients.world_control_client import WorldControlClient
from .environment import GYMNASIUM, Environment
from .reward_net import VAIRLRewardNet
from .utils import (
    fetch_demo_trajectories,
    trajectories_to_batches,
    trajectories_to_imitations,
)
from .vairl import VAIRL


class Trainer:
    """
    Creates environment and configures VAIRL with all necessary parameters for training.
    """

    def __init__(
        self,
        project: RoboGymProjectYaml,
        max_step_count: int | None = None,
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
        self.project = project
        self.max_step_count = max_step_count
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
        self.move_client = MoveClient()
        self.set_pose_client = SetPoseClient()
        self.world_control_client = WorldControlClient()
        self.controller_client = ControllerClient()
        self.localization_client = LocalizationClient()

        # Start up data collector client
        self.data_collector_client = DataCollectorClient()
        self.data_collector_client.establish_subscriptions(
            list(project["input_topics"].keys()),
        )

        # Fetch and process expert demonstrations
        self.demo_trajectories, determined_max_step_count, determined_max_vals = (
            fetch_demo_trajectories(
                self.project,
                self.expert_noise_std,
            )
        )
        self.max_step_count = (
            self.max_step_count
            if self.max_step_count
            else int(determined_max_step_count * 1.5)
        )
        self.max_vals = determined_max_vals
        self.demo_batches = trajectories_to_batches(self.demo_trajectories)
        self.demo_imitations = trajectories_to_imitations(self.demo_trajectories)
        self.flattened_demo_trajectories = rollout.flatten_trajectories(
            self.demo_imitations,
        )
        self.demos_batch_size = int(min(2048, len(self.flattened_demo_trajectories)))

        # Environment set up
        self.eval_environment = Environment(
            self.seed,
            self.max_step_count,
            self.max_vals,
            self.project["tensor_spec"]["input_features"],
            self.project["random_spawn_space"],
            self.data_collector_client,
            self.move_client,
            self.set_pose_client,
            self.controller_client,
            self.localization_client,
            True,
        )

        self.register_env()

        # TODO: Configure metric outputs

    def train(self):  # TODO: Incorporate metrics into training loop.
        """
        Train the VAIRL algorithm.
        """
        self._ready_simulation()

        # Create training environment
        train_venv = make_vec_env(
            self.env_id,
            rng=np.random.default_rng(self.seed),
            n_envs=1,
            env_make_kwargs={
                "seed": self.seed,
                "max_step_count": self.max_step_count,
                "max_vals": self.max_vals,
                "input_features": self.project["tensor_spec"]["input_features"],
                "random_spawn_space": self.project["random_spawn_space"],
                "data_collector_client": self.data_collector_client,
                "move_client": self.move_client,
                "set_pose_client": self.set_pose_client,
                "controller_client": self.controller_client,
                "localization_client": self.localization_client,
                "initialized": True,
            },
        )

        # Generator set up
        generator = TRPO(
            policy="MlpPolicy",
            env=train_venv,
            learning_rate=self.generator_learning_rate,
            batch_size=2048,
            gamma=0.99,
            gae_lambda=0.95,
            target_kl=0.01,
            verbose=True,
            seed=self.seed,
        )

        # Reward Net set up
        reward_net = VAIRLRewardNet(
            observation_space=train_venv.observation_space,
            action_space=train_venv.action_space,
            normalize_input_layer=RunningNorm,
            z_size=self.z_size,
            e_hidden_size=self.e_hidden_size,
            gamma=self.gamma,
        )

        # VAIRL algorithm set up
        vairl = VAIRL(
            demonstrations=self.flattened_demo_trajectories,
            demo_batch_size=self.demos_batch_size,
            venv=train_venv,
            gen_algo=generator,
            reward_net=reward_net,
            beta=0.0,
            i_c=self.i_c,
            beta_step_size=self.beta_step_size,
            disc_lr=self.discriminator_learning_rate,
            allow_variable_horizon=True,
        )

        train_discriminator_flag = True

        for episode in range(self.num_episodes):

            reward_net.eval()

            gen_data, reward_mean, reward_std = self.generate_generator_trajectories(
                generator,
                reward_net,
            )

            gen_batch = trajectories_to_batches(gen_data)

            # Train Generator
            vairl.train_gen(total_timesteps=self.rollout_steps)

            # Train Reward Net
            reward_net.train()

            # Train Discriminator
            if train_discriminator_flag:
                _disc_stats = vairl.train_disc(
                    expert_samples=self.demo_batches,
                    gen_samples=gen_batch,
                )
            else:
                _disc_stats = {"loss": 0.0, "kl": 0.0, "beta": float(vairl.beta)}

        # Finished training
        self._stop_simulation()
        self.data_collector_client.get_logger().info("FINISHED TRAINING")

    def register_env(self) -> None:
        try:
            if GYMNASIUM:
                from gymnasium.envs.registration import register
            else:
                from gym.envs.registration import register

            register(
                id=self.env_id,
                entry_point=Environment,
                max_episode_steps=self.max_step_count,
            )
        except Exception:
            # Already registered or gym registry unavailable; ignore
            pass

    def generate_generator_trajectories(
        self,
        generator: TRPO,
        reward_net: VAIRLRewardNet,
        n_trajs: int = 1,  # 20
    ):
        """
        Generate an array of trajectories and return the reward mean and standard deviation from the trajectories.
        """
        trajectories = []
        rewards = []

        for _ in range(n_trajs):

            if GYMNASIUM:
                state, _ = self.eval_environment.reset()
            else:
                state = self.eval_environment.reset()

            traj = []
            reward_accumulated = 0.0

            for _ in range(self.max_step_count):

                action, _ = generator.predict(state, deterministic=False)
                action = np.asarray(action, dtype=np.float32)

                if GYMNASIUM:
                    next_state, _env_reward, terminated, truncated, _ = (
                        self.eval_environment.step(action)
                    )
                    done = terminated or truncated
                else:
                    next_state, _env_reward, done, _ = self.eval_environment.step(
                        action,
                    )

                with torch.no_grad():
                    s_t = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
                    a_t = torch.tensor(action, dtype=torch.float32).unsqueeze(0)
                    s_next_t = torch.tensor(next_state, dtype=torch.float32).unsqueeze(
                        0,
                    )
                    d_t = torch.tensor([float(done)], dtype=torch.float32).unsqueeze(1)
                    reward = reward_net(s_t, a_t, s_next_t, d_t).item()

                reward_accumulated += reward
                traj.append((state.copy(), action.copy(), next_state.copy()))

                state = next_state

                if done:
                    if GYMNASIUM:
                        state, _ = self.eval_environment.reset()
                    else:
                        state = self.eval_environment.reset()

            rewards.append(reward_accumulated)
            trajectories.append(traj)

        return trajectories, float(np.mean(rewards)), float(np.std(rewards))

    def _ready_simulation(self):
        """
        Starts up simulation with controls and localization.
        """
        self._ready_gazebo()
        self.world_control_client.play_simulation()
        self.localization_client.start_localization()
        self.controller_client.start_controller()

    def _stop_simulation(self):
        """
        Stops the simulation and resets controls.
        """
        self.localization_client.reset_localization()
        self.controller_client.reset_controller()
        self.world_control_client.pause_simulation()

    def _ready_gazebo(headless: bool = False):
        """
        Configure environment variables and hints for faster Gazebo RL training.
        """
        # Run as fast as possible
        os.environ["GAZEBO_REAL_TIME_UPDATE_RATE"] = "0"

        # Reduce rendering load
        if headless:
            os.environ["DISPLAY"] = ""  # disables GUI
            os.environ["QT_QPA_PLATFORM"] = "offscreen"

        # Reduce sensor overhead (optional)
        os.environ["GAZEBO_SENSOR_NOISE"] = "0"
