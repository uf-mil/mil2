import csv
import queue
import threading
from pathlib import Path

import numpy as np
import pandas as pd

from ..filesystem import get_demo_dir_path
from ..types import Coord4D, RoboGymDemoYaml, RoboGymProjectYaml, StateActionPair
from ..utils import extract_selected_state_features


class AsyncCSVWriter:
    """
    Class responsible for writing to CSVs in the background.
    """

    def __init__(
        self,
        project: RoboGymProjectYaml,
        demo: RoboGymDemoYaml,
        flush_size: int = 1,
    ):

        self.q = queue.Queue()

        self.project = project
        self.demo = demo
        self.flush_size = flush_size

        self.demo_dir_path = get_demo_dir_path(project, demo)

        self.numerical_state_csv = (
            self.demo_dir_path / "data" / "numerical" / "data.csv"
        )
        self.action_csv = self.demo_dir_path / "data" / "actions" / "data.csv"

        self.unordered_sets = self.get_paths_for_unordered_sets()
        self.images = self.get_paths_for_images()

        self.abstract_data_counters = {}

        self._stop_event = threading.Event()

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def record(self, state: dict, action: dict) -> None:
        """
        Add the state action pair to the queue.
        """
        step = (state, action)

        self.q.put(step)

    def fetch_state_column_values(self, column: str) -> list:
        """
        Returns all the values for a certain column.
        """
        # NOTE: Images auto save so there is no need to pass them.
        df = pd.read_csv(self.numerical_state_csv)
        return df[column].values

    def fetch_state_series(self) -> dict[str, list[float]]:
        """
        Return all saved state columns as ordered numeric series.
        """
        try:
            df = pd.read_csv(self.numerical_state_csv)
        except (FileNotFoundError, pd.errors.EmptyDataError):
            return {}

        state_columns = self.project["tensor_spec"]["input_features"]
        return {
            column: [float(value) for value in df[column].dropna().tolist()]
            for column in state_columns
            if column in df.columns
        }

    def fetch_steps(self) -> list[Coord4D]:
        """
        Retrieve the list of steps taken.
        """
        df = pd.read_csv(self.action_csv)
        poses = df[["x", "y", "z", "yaw"]]

        return [tuple(row) for row in poses.to_numpy()]

    def close(self) -> None:
        """
        Stop the writer and flush out the remaining data.
        """
        self._stop_event.set()
        self.thread.join

    def clear_all_data(self, i_row: int | None = None) -> None:
        """
        Clear all data collected.
        """

        # Handle CSVs
        numerical_df = pd.read_csv(self.numerical_state_csv)
        action_df = pd.read_csv(self.action_csv)

        if i_row is not None:
            new_numerical_df = numerical_df.iloc[:i_row].copy()
            new_action_df = action_df.iloc[:i_row].copy()
        else:
            new_numerical_df = pd.DataFrame(columns=numerical_df.columns)
            new_action_df = pd.DataFrame(columns=action_df.columns)

        new_numerical_df.to_csv(self.numerical_state_csv, index=False)
        new_action_df.to_csv(self.action_csv, index=False)

        # Handle unordered set folders
        for name, topic_dir in self.unordered_sets.items():
            if not topic_dir.exists():
                continue

            files = sorted(topic_dir.glob("data_*.npy"))

            if i_row is None:
                # Delete everything
                for f in files:
                    f.unlink()
                self.abstract_data_counters[name] = 0
            else:
                # Keep only up to i_row
                for idx, f in enumerate(files):
                    if idx >= i_row:
                        f.unlink()

                self.abstract_data_counters[name] = min(i_row, len(files))

        # Handle image folders
        for name, img_dir in self.images.items():
            if not img_dir.exists():
                continue

            # supports img_0.jpg, img_1.png, etc.
            files = sorted(img_dir.glob("img_*.*"))

            if i_row is None:
                for f in files:
                    f.unlink()
            else:
                for idx, f in enumerate(files):
                    if idx >= i_row:
                        f.unlink()

    def set_new_paths(self, project: RoboGymProjectYaml, demo: RoboGymDemoYaml) -> None:
        """
        Set new paths for the csv files.
        """
        self.project = project
        self.demo = demo

        self.demo_dir_path = get_demo_dir_path(project, demo)

        self.numerical_state_csv = (
            self.demo_dir_path / "data" / "numerical" / "data.csv"
        )
        self.action_csv = self.demo_dir_path / "data" / "actions" / "data.csv"

        self.unordered_sets = self.get_paths_for_unordered_sets()
        self.images = self.get_paths_for_images()

    def get_paths_for_unordered_sets(self) -> dict[str, Path]:
        """
        Retrieves the path to all the unordered sets.
        """
        input_non_numeric_topics = self.project.get("input_non_numeric_topics", {})

        paths = {}

        for topic, data_list in input_non_numeric_topics.items():

            for data in data_list:
                
                if data["data_type"] == "unordered_set":

                    paths[f"{topic}:{data['field_path']}"] = (
                        self.demo_dir_path
                        / "data"
                        / (
                            topic.strip("/").replace("/", "_")
                            + "_"
                            + data["field_path"].replace(".", "_")
                        )
                    )

        return paths

    def get_paths_for_images(self) -> dict[str, Path]:
        """
        Retrieves the path to all image folders.
        """
        input_non_numeric_topics = self.project.get("input_non_numeric_topics", {})

        paths = {}

        for topic, data_list in input_non_numeric_topics.items():

            for data in data_list:

                if data["data_type"] == "image":

                    field_path = data["field_path"]

                    data_path = topic if field_path == "data" else f"{topic}:{field_path}"

                    paths[data_path] = (
                        self.demo_dir_path
                        / "data"
                        / (
                            topic.strip("/").replace("/", "_")
                            + (
                                ("_" + data["field_path"].replace(".", "_"))
                                if data["field_path"] != "data"
                                else ""
                            )
                        )
                    )

        return paths

    def _worker(self) -> None:
        """
        Background thread that writes data in batches.
        """
        buffer = []

        while not self._stop_event.is_set() or not self.q.empty():

            # Get step
            try:
                step = self.q.get()
            except queue.Empty:
                step = None

            if step is None:
                continue

            buffer.append(step)

            # Write buffer into respective CSVs if buffer overflows
            if len(buffer) >= self.flush_size:
                self._flush(buffer)
                buffer.clear()

        # Write buffer on close
        if buffer:
            self.flush(buffer)

    def _flush(self, buffer: list[StateActionPair]):
        """
        Write state and action data to CSVs.
        """
        # Extract numerical state data
        state_buffer = [sa_pair[0] for sa_pair in buffer]
        feature_names = self.project["tensor_spec"]["input_features"]
        numeric_state_buffer = [
            extract_selected_state_features(state, feature_names)
            for state in state_buffer
        ]

        state_fieldnames = feature_names

        # Extract numerical action (at the moment only movement)
        action_buffer = [sa_pair[1] for sa_pair in buffer]

        # Extract unordered set data
        unordered_set_feature_names = [
            f"{topic}:{data['field_path']}"
            for topic, data_list in self.project.get("input_non_numeric_topics", {}).items()
            for data in data_list
            if data["data_type"] == "unordered_set"
        ]
        unordered_set_state_buffer = [
            extract_selected_state_features(state, unordered_set_feature_names)
            for state in state_buffer
        ]

        # Write numerical state data
        with open(self.numerical_state_csv, "a", newline="") as f_state:
            writer = csv.DictWriter(
                f_state,
                fieldnames=state_fieldnames,
                extrasaction="ignore",
            )
            writer.writerows(numeric_state_buffer)

        # Write unordered set state data
        for name in unordered_set_feature_names:
            topic_dir = self.unordered_sets[name]
            topic_dir.mkdir(parents=True, exist_ok=True)

            if name not in self.abstract_data_counters:
                try:
                    self.abstract_data_counters[name] = len(
                        list(topic_dir.iterdir()),
                    )
                except:
                    self.abstract_data_counters[name] = 0

            for state_idx, state_features in enumerate(unordered_set_state_buffer):

                data = state_features.get(name)

                index = self.abstract_data_counters[name]
                file_path = topic_dir / f"data_{index}.npy"

                np.save(file_path, np.array(data, dtype=object))

                self.abstract_data_counters[name] += 1

        # Write action data
        with open(self.action_csv, "a", newline="") as f_action:
            writer = csv.DictWriter(
                f_action,
                fieldnames=action_buffer[0].keys(),
                extrasaction="ignore",
            )
            writer.writerows(action_buffer)
