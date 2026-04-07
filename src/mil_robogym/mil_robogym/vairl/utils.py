from pathlib import Path

import numpy as np
import pandas as pd
from imitation.data.types import Trajectory

from mil_robogym.data_collection.filesystem import get_project_dir_path
from mil_robogym.data_collection.types import RoboGymProjectYaml


def _add_noise(action: np.ndarray, noise_std: float) -> np.ndarray:
    if noise_std <= 0:
        return action
    return action + np.random.normal(scale=noise_std, size=action.shape)


def fetch_demo_trajectories(project: RoboGymProjectYaml, noise_std: float):
    """
    Go through all demo folders in project, compose trajectories as 3-tuple s.t. (s_t, a_t+1, s_t+1).

    Returns the trajectories and the max number of steps and max step size taken from all demos.
    """
    demos_dir = get_project_dir_path(project) / "demos"

    trajectories = []
    max_number_of_steps = 0
    max_vals = np.zeros(4, dtype=np.float32)

    relative_abstract_directories = _get_relative_abstract_directories(project)

    for demo_dir in demos_dir.iterdir():

        if demo_dir.is_dir():

            # Paths to CSVs
            numerical_csv = demo_dir / "data" / "numerical" / "data.csv"
            actions_csv = demo_dir / "data" / "actions" / "data.csv"

            # Load data
            numerical_df = pd.read_csv(numerical_csv)
            actions_df = pd.read_csv(actions_csv).loc[
                :,
                ["delta_x", "delta_y", "delta_z", "delta_yaw"],
            ]

            # Collect max values per dimension
            demo_max = np.abs(actions_df.values).max(axis=0)
            max_vals = np.maximum(max_vals, demo_max)

            # Convert to numpy
            states = numerical_df.values
            actions = actions_df.values

            num_steps = len(states)

            # Get paths to abstract data
            abstract_data_per_feature = []

            for rel_path in relative_abstract_directories:

                absolute_path = demo_dir / "data" / rel_path

                files = sorted(absolute_path.iterdir())
                file_paths = [str(f) for f in files]

                abstract_data_per_feature.append(file_paths[:num_steps])

            if abstract_data_per_feature:

                abstract_matrix = np.array(abstract_data_per_feature, dtype=object).T

                states = np.concatenate([states, abstract_matrix], axis=1)

            # Align states and actions: (s_t, a_t+1)
            states_t = states[:-1]
            next_states = states[1:]
            actions_t = actions[1:]

            # Add noise to actions
            actions_t = [_add_noise(a, noise_std) for a in actions_t]

            # Check states and actions are the same length
            assert (
                len(states_t) == len(actions_t) == len(next_states)
            ), f"Mismatch in {demo_dir.name}: number of states and actions must match (s: {len(states_t)}, a: {len(actions_t)}, s_+1: {len(next_states)})"

            # Build trajectory
            trajectory = list(zip(states_t, actions_t, next_states))

            # Store trajectory
            trajectories.append(trajectory)

            # Compute max number of steps
            max_number_of_steps = max(max_number_of_steps, len(trajectory))

    return trajectories, max_number_of_steps, max_vals


def trajectories_to_batches(
    trajs: list[list[tuple[np.ndarray, np.ndarray, np.ndarray]]],
):
    obs_list = []
    acts_list = []
    next_obs_list = []
    dones_list = []

    for traj in trajs:
        for i, (s, a, s_next) in enumerate(traj):
            obs_list.append(s)
            acts_list.append(a)
            next_obs_list.append(s_next)
            dones_list.append(1.0 if i == len(traj) - 1 else 0.0)

    return {
        "obs": np.asarray(obs_list, dtype=np.float32),
        "acts": np.asarray(acts_list, dtype=np.float32),
        "next_obs": np.asarray(next_obs_list, dtype=np.float32),
        "dones": np.asarray(dones_list, dtype=np.float32),
    }


def trajectories_to_imitations(
    trajectories: list[list[tuple[np.ndarray, np.ndarray, np.ndarray]]],
):
    trajs = []
    for traj in trajectories:
        obs = [step[0] for step in traj]
        obs.append(traj[-1][2])
        acts = [step[1] for step in traj]
        obs_arr = np.asarray(obs, dtype=np.float32)
        acts_arr = np.asarray(acts, dtype=np.float32)
        trajs.append(
            Trajectory(
                obs=obs_arr,
                acts=acts_arr,
                infos=None,
                terminal=True,
            ),
        )
    return trajs


def _get_relative_abstract_directories(project: RoboGymProjectYaml) -> list[Path]:

    input_non_numeric_topics = project.get("input_non_numeric_topics", {})

    relative_paths = []

    for topic, data_list in input_non_numeric_topics:

        parsed_topic = topic.strip("/").replace("/", "_")
        
        for data in data_list:
            
            parsed_field = (
                ""
                if data["field_path"] == "data"
                and data["data_type"] == "image"
                else data["field_path"]
            )

            relative_paths.append(
                Path(parsed_topic + (f"_{parsed_field}" if parsed_field else "")),
            )

    return relative_paths
