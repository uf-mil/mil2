import os
import threading
from concurrent.futures import Future
from datetime import datetime
from pathlib import Path
from typing import Callable

import numpy as np
import torch
from imitation.data import rollout
from imitation.util.networks import RunningNorm
from imitation.util.util import make_vec_env
from rosidl_runtime_py.utilities import get_message
from sb3_contrib import TRPO

from mil_robogym.data_collection.filesystem import (
    _format_agent_timestamp,
    get_training_project_dir_paths,
)
from mil_robogym.data_collection.types import RoboGymProjectYaml

from ..clients.controller_client import ControllerClient
from ..clients.data_collector_client import DataCollectorClient
from ..clients.localization_client import LocalizationClient
from ..clients.move_client import MoveClient
from ..clients.set_pose_client import SetPoseClient
from ..clients.world_control_client import WorldControlClient
from .deep_set import DeepSet
from .environment import GYMNASIUM, Environment
from .generator_metrics import extract_latest_generator_metrics
from .image_encoder import CNNEncoder
from .metrics import TrainingMetricsSession
from .reward_net import VAIRLRewardNet
from .training_settings import DEFAULT_TRAINING_SETTINGS
from .utils import (
    fetch_demo_trajectories,
    interpret_abstract_data,
    trajectories_to_batches,
    trajectories_to_imitations,
)
from .vairl import VAIRL

GENERATOR_MODEL_FILE_NAME = "generator_model.zip"


class Trainer:
    """
    Creates environment and configures VAIRL with all necessary parameters for training.
    """

    def __init__(
        self,
        project: RoboGymProjectYaml,
        max_step_count: int | None = None,
        num_episodes: int = DEFAULT_TRAINING_SETTINGS["num_episodes"],
        rollout_steps: int = DEFAULT_TRAINING_SETTINGS["rollout_steps"],
        generator_learning_rate: float = DEFAULT_TRAINING_SETTINGS[
            "generator_learning_rate"
        ],
        discriminator_learning_rate: float = DEFAULT_TRAINING_SETTINGS[
            "discriminator_learning_rate"
        ],
        z_size: int = DEFAULT_TRAINING_SETTINGS["z_size"],
        e_hidden_size: int = DEFAULT_TRAINING_SETTINGS["e_hidden_size"],
        i_c: float = DEFAULT_TRAINING_SETTINGS["i_c"],
        beta_step_size: float = DEFAULT_TRAINING_SETTINGS["beta_step_size"],
        gamma: float = DEFAULT_TRAINING_SETTINGS["gamma"],
        save_every: int = DEFAULT_TRAINING_SETTINGS["save_every"],
        seed: int = DEFAULT_TRAINING_SETTINGS["seed"],
        env_id: str = "VairlROS2-v0",
        expert_noise_std: float = DEFAULT_TRAINING_SETTINGS["expert_noise_std"],
        progress_callback: Callable[[dict[str, object]], None] | None = None,
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
        self.progress_callback = progress_callback
        self._stop_requested = threading.Event()
        self._abort_requested = threading.Event()
        self._stopped_early = False
        self._aborted = False

        # ROS2 components
        self.move_client = MoveClient()
        self.set_pose_client = SetPoseClient()
        self.world_control_client = WorldControlClient()
        self.controller_client = ControllerClient()
        self.localization_client = LocalizationClient()

        # Start up data collector client
        self.data_collector_client = DataCollectorClient()
        self.data_collector_client.establish_subscriptions(project)

        # Fetch and process expert demonstrations
        self.does_contain_abstract_data = (
            self.project.get("input_non_numeric_topics", {}) != {}
        )
        self.demo_trajectories, determined_max_step_count, determined_max_vals = (
            fetch_demo_trajectories(
                self.project,
                self.expert_noise_std,
            )
        )
        self.demos_batch_size = int(
            min(2048, len(self.demo_trajectories)),
        )

        self.max_step_count = (
            self.max_step_count
            if self.max_step_count
            else int(determined_max_step_count * 1.5)
        )
        self.max_vals = determined_max_vals

        # Only fetch this data if no abstract data is being used to train the agent
        if not self.does_contain_abstract_data:
            self.demo_batches = trajectories_to_batches(self.demo_trajectories)
            self.demo_imitations = trajectories_to_imitations(self.demo_trajectories)
            self.flattened_demo_trajectories = rollout.flatten_trajectories(
                self.demo_imitations,
            )
            self.external_architecure = []
        else:
            self.demo_batches = None
            self.demo_imitations = None
            self.flattened_demo_trajectories = None

            # Load in external architectures for abstract data types
            self.external_architecture = self._create_external_architecture()

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

    def train(self):
        """
        Train the VAIRL algorithm.
        """
        if not hasattr(self, "_stop_requested"):
            self._stop_requested = threading.Event()
        if not hasattr(self, "_abort_requested"):
            self._abort_requested = threading.Event()
        if not hasattr(self, "_stopped_early"):
            self._stopped_early = False
        if not hasattr(self, "_aborted"):
            self._aborted = False

        metrics_session = TrainingMetricsSession(
            save_callback=self._emit_progress_event,
        )
        session_closed = False
        caught_exception: Exception | None = None
        last_checkpoint_save: Future[list[Path]] | None = None
        self._ready_simulation()

        try:
            # Create training environment
            train_venv = make_vec_env(
                self.env_id,
                rng=np.random.default_rng(self.seed),
                n_envs=1,
                env_make_kwargs={
                    "seed": self.seed,
                    "max_step_count": self.max_step_count,
                    "max_vals": self.max_vals,
                    "input_features": self.project["tensor_spec"][
                        "input_features"
                    ],  # TODO: Might be best to pass in project
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
                batch_size=self.demos_batch_size,
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
                if self._abort_requested.is_set():
                    self._stopped_early = True
                    self._aborted = True
                    break
                if self._stop_requested.is_set():
                    self._stopped_early = True
                    break

                if self.does_contain_abstract_data:
                    # Recalculate values for abstract data
                    pass

                # Interpret abstract data if any
                if self.does_contain_abstract_data:
                    interpreted_trajectories = interpret_abstract_data(
                        self.demo_trajectories,
                        self.external_architecture,
                    )
                    self.demo_batches = trajectories_to_batches(
                        interpreted_trajectories,
                    )
                    self.demo_imitations = trajectories_to_imitations(
                        self.demo_trajectories,
                    )
                    self.flattened_demo_trajectories = rollout.flatten_trajectories(
                        self.demo_imitations,
                    )

                reward_net.eval()

                gen_data, reward_mean, reward_std = (
                    self.generate_generator_trajectories(
                        generator,
                        reward_net,
                        self.demos_batch_size,
                    )
                )

                gen_batch = trajectories_to_batches(gen_data)

                # Train Generator
                vairl.train_gen(total_timesteps=self.rollout_steps)
                generator_stats = extract_latest_generator_metrics(vairl.logger)

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

                episode_number = episode + 1
                metrics_session.record_episode(
                    episode=episode_number,
                    reward_mean=reward_mean,
                    reward_std=reward_std,
                    disc_stats=_disc_stats,
                    extra_metrics=generator_stats,
                )
                self._emit_progress_event(
                    {
                        "type": "metrics_updated",
                        "episode": episode_number,
                        "num_episodes": self.num_episodes,
                        "metrics": metrics_session.snapshot(),
                    },
                )
                if self._abort_requested.is_set():
                    self._stopped_early = True
                    self._aborted = True
                    break
                if self._stop_requested.is_set():
                    self._stopped_early = True
                    break
                if self.save_every > 0 and episode_number % self.save_every == 0:
                    last_checkpoint_save = self._save_generator_model(
                        generator,
                        metrics_session,
                        checkpoint_episode=episode_number,
                    )

            if self._abort_requested.is_set():
                metrics_session.close()
                session_closed = True
                self.data_collector_client.get_logger().info("ABORTED TRAINING")
                if last_checkpoint_save is not None:
                    return last_checkpoint_save.result()
                return []

            final_saved_agent_dirs = self._save_generator_model(
                generator,
                metrics_session,
                is_final=True,
            ).result()
            metrics_session.close()
            session_closed = True

            # Finished training
            self.data_collector_client.get_logger().info("FINISHED TRAINING")
            return final_saved_agent_dirs
        except Exception as exc:
            caught_exception = exc
            raise
        finally:
            close_error: Exception | None = None
            if not session_closed:
                try:
                    metrics_session.close()
                except Exception as exc:
                    close_error = exc

            stop_error: Exception | None = None
            try:
                self._stop_simulation()
            except Exception as exc:
                stop_error = exc

            if caught_exception is None:
                if close_error is not None:
                    raise close_error
                if stop_error is not None:
                    raise stop_error

    def _create_external_architecture(self):
        """
        Iterate through project yaml and set up CNN or DeepSet accordingly.
        """

        external_architecture = []
        input_non_numeric_topics = self.project.get("input_non_numeric_topics", {})

        for topic, data_list in input_non_numeric_topics.items():

            for data in data_list:

                if data["data_type"] == "image":
                    external_architecture.append(CNNEncoder())

                elif data["data_type"] == "unordered_set":
                    ros_type = data["ros_type"]

                    try:
                        # Extract inner type from sequence<...>
                        if "sequence<" in ros_type:
                            inner_type = ros_type.replace("sequence<", "").replace(
                                ">",
                                "",
                            )
                        else:
                            inner_type = ros_type

                        # Get ROS message class
                        msg_class = get_message(inner_type)

                        # Create dummy instance
                        dummy_obj = msg_class()

                        # Define output shape (can later move to YAML)
                        output_dim = data.get("output_dim", 32)

                        external_architecture.append(
                            DeepSet(obj=dummy_obj, output_shape=output_dim),
                        )

                    except Exception as e:
                        raise RuntimeError(
                            f"Failed to initialize DeepSet for topic '{topic}' with type '{ros_type}': {e}",
                        )

                else:
                    raise ValueError(f"Unsupported data_type: {data['data_type']}")

        return external_architecture

    def _build_agent_name(
        self,
        created_at: datetime,
        *,
        checkpoint_episode: int | None = None,
        is_final: bool = False,
    ) -> str:
        base_name = _format_agent_timestamp(created_at)
        if is_final:
            return f"{base_name}_final"
        if checkpoint_episode is not None:
            return f"{base_name}_ep_{checkpoint_episode:04d}"
        return base_name

    def _resolve_unique_agent_name(
        self,
        agent_name: str,
        project_dirs: list[Path],
    ) -> str:
        def exists(candidate: str) -> bool:
            return any(
                (project_dir / "agents" / candidate).exists()
                for project_dir in project_dirs
            )

        if not exists(agent_name):
            return agent_name

        suffix = 2
        while True:
            candidate = f"{agent_name}_run_{suffix:02d}"
            if not exists(candidate):
                return candidate
            suffix += 1

    def _save_generator_model(
        self,
        generator: TRPO,
        metrics_session: TrainingMetricsSession,
        *,
        checkpoint_episode: int | None = None,
        is_final: bool = False,
    ) -> Future[list[Path]]:
        created_at = datetime.now()
        project_dirs = get_training_project_dir_paths(self.project)
        agent_name = self._resolve_unique_agent_name(
            self._build_agent_name(
                created_at,
                checkpoint_episode=checkpoint_episode,
                is_final=is_final,
            ),
            project_dirs,
        )
        num_demos = len(self.demo_trajectories)
        return metrics_session.enqueue_agent_save(
            generator=generator,
            project_dirs=project_dirs,
            num_demos=num_demos,
            created_at=created_at,
            model_file_name=GENERATOR_MODEL_FILE_NAME,
            agent_name=agent_name,
            checkpoint_episode=checkpoint_episode,
            training_settings=self._resolved_training_settings(),
        )

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
        reward_net_device = next(reward_net.parameters()).device

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
                    s_t = torch.as_tensor(
                        state,
                        dtype=torch.float32,
                        device=reward_net_device,
                    ).unsqueeze(0)
                    a_t = torch.as_tensor(
                        action,
                        dtype=torch.float32,
                        device=reward_net_device,
                    ).unsqueeze(0)
                    s_next_t = torch.as_tensor(
                        next_state,
                        dtype=torch.float32,
                        device=reward_net_device,
                    ).unsqueeze(0)
                    d_t = torch.as_tensor(
                        [float(done)],
                        dtype=torch.float32,
                        device=reward_net_device,
                    ).unsqueeze(1)
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

    def _resolved_training_settings(self) -> dict[str, int | float | None]:
        return {
            "num_episodes": self.num_episodes,
            "rollout_steps": self.rollout_steps,
            "generator_learning_rate": self.generator_learning_rate,
            "discriminator_learning_rate": self.discriminator_learning_rate,
            "z_size": self.z_size,
            "e_hidden_size": self.e_hidden_size,
            "i_c": self.i_c,
            "beta_step_size": self.beta_step_size,
            "gamma": self.gamma,
            "save_every": self.save_every,
            "seed": self.seed,
            "max_step_count": self.max_step_count,
            "expert_noise_std": self.expert_noise_std,
        }

    def _emit_progress_event(self, event: dict[str, object]) -> None:
        progress_callback = getattr(self, "progress_callback", None)
        if progress_callback is not None:
            progress_callback(event)

    def request_stop(self) -> None:
        """Ask the trainer to stop at the next safe boundary."""
        if not hasattr(self, "_stop_requested"):
            self._stop_requested = threading.Event()
        self._stop_requested.set()

    def request_abort(self) -> None:
        """Abort the run and discard any unsaved final checkpoint."""
        if not hasattr(self, "_abort_requested"):
            self._abort_requested = threading.Event()
        if not hasattr(self, "_stop_requested"):
            self._stop_requested = threading.Event()
        self._abort_requested.set()
        self._stop_requested.set()

    def was_stopped(self) -> bool:
        """Report whether the loop exited early due to a stop request."""
        return bool(getattr(self, "_stopped_early", False))

    def was_aborted(self) -> bool:
        """Report whether the run exited via the abort path."""
        return bool(getattr(self, "_aborted", False))

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
