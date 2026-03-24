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
    """
    demos_dir = get_project_dir_path(project) / "demos"

    trajectories = []

    for demo_dir in demos_dir.iterdir():

        if demo_dir.is_dir():

            # Paths to CSVs
            numerical_csv = demo_dir / "data" / "numerical" / "data.csv"
            actions_csv = demo_dir / "data" / "actions" / "data.csv"

            # Load data
            numerical_df = pd.read_csv(numerical_csv)
            actions_df = pd.read_csv(actions_csv)

            # Convert to numpy
            states = numerical_df.values
            actions = actions_df.values

            # Align states and actions: (s_t, a_t+1)
            states = states[:-1]
            next_states = states[1:]
            actions = actions[1:]

            # Add noise to actions
            actions = [_add_noise(a, noise_std) for a in actions]

            # Check states and actions are the same length
            assert (
                len(states) == len(actions) == len(next_states)
            ), f"Mismatch in {demo_dir.name}: number of states and actions must match (s: {len(states)}, a: {len(actions)}, s_+1: {len(next_states)})"

            # Build trajectory
            trajectory = list(zip(states, actions, next_states))

            # Store trajectory
            trajectories.append(trajectory)

    return trajectories


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
