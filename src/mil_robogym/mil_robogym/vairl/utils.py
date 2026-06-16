import json
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import torch
from imitation.data.types import Trajectory

from mil_robogym.data_collection.filesystem import get_project_dir_path
from mil_robogym.data_collection.types import RoboGymProjectYaml

from .deep_set import DeepSet
from .image_encoder import CNNEncoder


def _add_noise(action: np.ndarray, noise_std: float) -> np.ndarray:
    if noise_std <= 0:
        return action
    return action + np.random.normal(scale=noise_std, size=action.shape)


def _extract_number(path: Path) -> int:
    name = path.stem  # e.g., "img_12" or "data_3"
    return int(name.split("_")[1])


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
            try:
                numerical_df = pd.read_csv(numerical_csv)
            except pd.errors.EmptyDataError:
                numerical_df = pd.DataFrame()

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

            num_steps = len(actions)

            # Get paths to abstract data
            abstract_data_per_feature = []

            for rel_path in relative_abstract_directories:

                absolute_path = demo_dir / "data" / rel_path

                files = sorted(absolute_path.iterdir(), key=_extract_number)
                file_paths = [str(f) for f in files]

                abstract_data_per_feature.append(file_paths[:num_steps])

            if abstract_data_per_feature:

                abstract_matrix = np.stack(abstract_data_per_feature, axis=1)

                if states.size > 0:
                    states = np.concatenate([states, abstract_matrix], axis=1)
                else:
                    states = abstract_matrix

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


def load_file(path, device="cpu") -> np.ndarray:  # TODO: Pass in correct device
    if path.endswith(".jpg"):
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        return img

    elif path.endswith(".json"):
        with open(path) as f:
            return json.load(f, object_pairs_hook=OrderedDict)

    else:
        raise ValueError(f"Unsupported file type: {path}")


def interpret_state_data(
    state: list[any],
    models: list[DeepSet, CNNEncoder],
) -> list[int | float | complex]:
    """
    Passes strictly ordered state data and passes non numerical data through encoder models.
    """
    if not models:
        return np.array(state)

    num_abstract_values = len(models)
    index = len(state) - num_abstract_values

    abstract_data = state[-num_abstract_values:]

    for data, model in zip(abstract_data, models):

        if isinstance(data, list):
            data = np.array(data)

        # Run model
        with torch.no_grad():
            output = model(data)

        # Ensure NumPy output
        if isinstance(output, torch.Tensor):
            output = output.detach().cpu().numpy()

        state[index] = output.flatten()

        index += 1

    flattened_state = []

    for item in state:
        if isinstance(item, (list, tuple, np.ndarray)):
            flattened_state.extend(item)
        else:
            flattened_state.append(item)

    return np.array(flattened_state)


def interpret_abstract_data(
    trajs: list[
        list[tuple[np.ndarray, np.ndarray, np.ndarray]]
    ],  # state, action, next_state
    models: list[DeepSet | CNNEncoder],
) -> list[list[tuple[np.ndarray, np.ndarray, np.ndarray]]]:
    """
    Maps the data to its values by passing it through their respective neural net.
    """
    num_abstract_columns = len(models)

    # Flatten states
    flat_states = []
    flat_next_states = []
    traj_lengths = []

    for traj in trajs:
        traj_lengths.append(len(traj))
        for s, a, ns in traj:
            flat_states.append(s)
            flat_next_states.append(ns)

    flat_states = np.array(flat_states, dtype=object)
    flat_next_states = np.array(flat_next_states, dtype=object)

    # Split numeric vs abstract paths
    state_numeric = np.stack([s[:-num_abstract_columns] for s in flat_states])
    next_numeric = np.stack([s[:-num_abstract_columns] for s in flat_next_states])

    # Cache: {model_index: {path: embedding}}
    caches = [{} for _ in range(num_abstract_columns)]

    encoded_states = []
    encoded_next_states = []

    for i, model in enumerate(models):
        # Collect all paths
        state_paths = [s[-num_abstract_columns + i] for s in flat_states]
        next_paths = [s[-num_abstract_columns + i] for s in flat_next_states]

        all_paths = list(set(state_paths + next_paths))  # deduplicate

        # Load data
        loaded_data = []
        valid_paths = []

        for path in all_paths:
            try:
                data = load_file(path)
                loaded_data.append(data)
                valid_paths.append(path)
            except Exception as e:
                print(f"Skipping {path}: {e}")

        # Batch inference
        if len(loaded_data) > 0:
            # batch = np.stack(loaded_data)

            with torch.no_grad():
                embeddings = model(loaded_data)  # batch

            # Store in cache
            for path, emb in zip(valid_paths, embeddings):
                caches[i][path] = emb

        # Map back
        state_embeds = np.stack([caches[i][p] for p in state_paths])
        next_embeds = np.stack([caches[i][p] for p in next_paths])

        encoded_states.append(state_embeds)
        encoded_next_states.append(next_embeds)

    # Concatenate all encoded features
    encoded_states = np.concatenate(encoded_states, axis=1)
    encoded_next_states = np.concatenate(encoded_next_states, axis=1)

    # Rebuild states
    new_states = np.concatenate([state_numeric, encoded_states], axis=1)
    new_next_states = np.concatenate([next_numeric, encoded_next_states], axis=1)

    # Reconstruct trajectories
    result = []
    idx = 0

    for traj_len, traj in zip(traj_lengths, trajs):
        new_traj = []
        for j in range(traj_len):
            _, action, _ = traj[j]

            new_traj.append((new_states[idx], action, new_next_states[idx]))
            idx += 1

        result.append(new_traj)

    return result


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

    for topic, data_list in input_non_numeric_topics.items():

        parsed_topic = topic.strip("/").replace("/", "_")

        for data in data_list:

            parsed_field = (
                ""
                if data["field_path"] == "data" and data["data_type"] == "image"
                else data["field_path"]
            )

            relative_paths.append(
                Path(parsed_topic + (f"_{parsed_field}" if parsed_field else "")),
            )

    return relative_paths
