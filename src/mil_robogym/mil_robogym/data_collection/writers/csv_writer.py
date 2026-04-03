import csv
import queue
import threading

import pandas as pd

from ..filesystem import get_demo_dir_path
from ..types import Coord4D, RoboGymDemoYaml, RoboGymProjectYaml, StateActionPair
from ..utils import flatten_value


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
        df = pd.read_csv(self.numerical_state_csv)
        return df[column].values

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
        state_buffer = [sa_pair[0] for sa_pair in buffer]
        state_buffer = list(map(self._flatten_and_filter_state_fields, state_buffer))

        state_fieldnames = self.project["tensor_spec"]["input_features"]

        action_buffer = [sa_pair[1] for sa_pair in buffer]

        with open(self.numerical_state_csv, "a", newline="") as f_state:
            writer = csv.DictWriter(
                f_state,
                fieldnames=state_fieldnames,
                extrasaction="ignore",
            )
            writer.writerows(state_buffer)

        with open(self.action_csv, "a", newline="") as f_action:
            writer = csv.DictWriter(
                f_action,
                fieldnames=action_buffer[0].keys(),
                extrasaction="ignore",
            )
            writer.writerows(action_buffer)

    def _flatten_and_filter_state_fields(self, state: dict) -> dict:
        """
        Flatten dict into column names and keep only desired column names.
        """
        flattened_states = {}

        for topic, msg in state.items():

            temp = {}
            flatten_value(msg, "", temp)

            for key, value in temp.items():

                feature_name = f"{topic}:{key}"
                flattened_states[feature_name] = value

        features_allowed = set(self.project["tensor_spec"]["input_features"])

        return {k: v for k, v in flattened_states.items() if k in features_allowed}
