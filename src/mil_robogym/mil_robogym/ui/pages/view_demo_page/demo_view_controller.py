import json
import tkinter as tk
from tkinter import messagebox
from typing import Any

import numpy as np

from mil_robogym.clients.data_collector_client import DataCollectorClient
from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.set_pose_client import SetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.filesystem import edit_demo
from mil_robogym.data_collection.types import (
    Coord4D,
    RoboGymDemoYaml,
    RoboGymProjectYaml,
)
from mil_robogym.data_collection.utils import extract_selected_state_features
from mil_robogym.data_collection.writers.csv_writer import AsyncCSVWriter
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI

from .edit_demo_popup import EditDemoPopup


class DemoViewController:
    """
    Class that handles logic for a demo.
    """

    def __init__(self, view: tk.Frame, app):
        self.view = view
        self.app = app

        self.project: RoboGymProjectYaml | None = None
        self.demo: RoboGymDemoYaml | None = None
        self.steps: list[dict] = []

        self.coordinate_popup = None
        self.edit_demo_popup = None

        self.is_recording = False
        self.last_pose = None
        self.delay = -1

        self.get_pose_client = GetPoseClient()
        self.set_pose_client = SetPoseClient()
        self.world_control_client = WorldControlClient()
        self.data_collector = DataCollectorClient()
        self.csv_writer = None
        self.keyboard_controls_gui = KeyboardControlsGUI(
            self.view,
            self.pause_recording,
        )
        self.keyboard_controls_gui.hide()

    def set_context(
        self,
        project: dict[str, Any] | None = None,
        demo: dict[str, Any] | None = None,
    ) -> None:

        self.raw_project = project
        self.project = project.get("robogym_project", {})
        self.demo = demo.get("robogym_demo", {})

        if self.project and self.demo:

            self.delay = int((1.0 / self.demo["sampling_rate"]) * 1000)

            # Create and start services
            self.csv_writer = AsyncCSVWriter(self.project, self.demo)
            self.data_collector.establish_subscriptions(
                list(self.project["input_topics"].keys()),
            )

            # Configure UI Components
            self.view.header.project_title.config(text=f"{self.project['name']} >")
            self.view.header.demo_title.config(text=self.demo["name"])
            self.view.header.subtitle.config(
                text=(
                    f"Sampling rate: {self.demo['sampling_rate']} steps / sec | "
                    f"World: {self.project['world_file']}"
                ),
            )

            self.view.controls.random_position_button.config(
                state=(
                    tk.NORMAL
                    if self.project["random_spawn_space"]["enabled"]
                    else tk.DISABLED
                ),
            )

            self.view.steps.clear()
            x, y, z, yaw = self.demo["start_position"]
            self.view.steps.add_step((x, y, z, yaw), is_origin=True)

            # Load steps in from CSV files
            steps = self.csv_writer.fetch_steps()
            for step in steps:
                self.view.steps.add_step(step)

            # Configure data section graphs
            self.view.data_section.set_metric_order(
                self.project["tensor_spec"]["input_features"],
            )
            self.update_graph()

            # Set last pose
            if steps:
                # Disable random pose and preposition
                self.view.controls.preposition_button.config(state=tk.DISABLED)
                self.view.controls.random_position_button.config(state=tk.DISABLED)

                # Set last pose
                x, y, z, yaw = steps[-1]
                self.last_pose = (x, y, z, yaw)
            else:
                self.last_pose = (x, y, z, yaw)

            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def navigate_to_home(self, _event: tk.Event | None = None) -> None:

        # Check if pose index is not last and override data
        if self.view.steps.current_pose_index != len(self.view.steps.steps) - 1:
            self.csv_writer.clear_all_data(self.view.steps.current_pose_index)

        self.csv_writer.close()
        self._clean_components()
        self.app.show_page("start")

    def navigate_to_project(self, _event: tk.Event | None = None) -> None:

        # Check if pose index is not last and override data
        if self.view.steps.current_pose_index != len(self.view.steps.steps) - 1:
            self.csv_writer.clear_all_data(self.view.steps.current_pose_index)

        self.csv_writer.close()
        self._clean_components()
        self.app.show_page("view_project", project=self.raw_project)

    def move_model_to(self, coordinate: Coord4D) -> None:
        """
        Move the model to the provided position.
        """
        x, y, z, yaw = coordinate
        self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def start_recording(self) -> None:
        """
        Starts recording steps.
        """
        # Place sub in last position recorded
        if self.last_pose:
            x, y, z, yaw = self.last_pose
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

        # Check if pose index is not last and override data
        if self.view.steps.current_pose_index != len(self.view.steps.steps) - 1:
            self.csv_writer.clear_all_data(self.view.steps.current_pose_index)
            self.view.steps.destroy_undone_steps()
            self.update_graph()

        # Enable keyboard controls
        self.keyboard_controls_gui.show()

        # Start gazebo simulation
        self.world_control_client.play_simulation()

        # Disable the play button and enable the pause button
        self.view.controls.pause_button.config(state=tk.NORMAL)
        self.view.controls.undo_button.config(state=tk.DISABLED)
        self.view.controls.redo_button.config(state=tk.DISABLED)
        self.view.controls.play_button.config(state=tk.DISABLED)

        # Buffer sampling by 1 sample
        self.data_collector.buffer = True

        # Start sampling asynchronously
        self.is_recording = True

        # Delay sampling if mid session
        if len(self.view.steps.steps) > 1:
            self.view.after(self.delay, self._schedule_next_sample)
        else:
            self._schedule_next_sample()

    def pause_recording(self) -> None:
        """
        Pauses the recording of steps.
        """
        # Pause the gazebo environment
        self.world_control_client.pause_simulation()

        # Disable keyboard controls
        self.is_recording = False
        self.keyboard_controls_gui.hide()

        # Disable the pause button and enable the play button.
        self.view.controls.pause_button.config(state=tk.DISABLED)
        self.view.controls.undo_button.config(state=tk.NORMAL)
        self.view.controls.redo_button.config(state=tk.NORMAL)
        self.view.controls.play_button.config(state=tk.NORMAL)

    def preposition(self) -> None:
        """
        Enable keyboard controls to move the sub without recording steps.
        """
        # Display coordinate popup
        if self.coordinate_popup and self.coordinate_popup.win.winfo_exists():
            self.coordinate_popup.win.lift()
            self.coordinate_popup.win.focus_force()
        else:
            self.coordinate_popup = GrabCoordinatesPopup(
                self.view,
                self.get_pose_client.send_request,
                self._save_coordinate,
                num_coordinates=1,
            )

        # Enable keyboard controls
        self.keyboard_controls_gui.show()

        # Play simulation
        self.world_control_client.play_simulation()

        # Disable pause and play buttons
        self.view.controls.pause_button.config(state=tk.DISABLED)
        self.view.controls.play_button.config(state=tk.DISABLED)

    def set_random_origin(self) -> None:
        """
        Sets a random start position if a random spawn space was defined.
        """
        random_spawn_space = self.project["random_spawn_space"]

        c1 = random_spawn_space["coord1_4d"]
        c2 = random_spawn_space["coord2_4d"]

        if random_spawn_space["enabled"] and c1 and c2:
            low = np.minimum(c1, c2)
            high = np.maximum(c1, c2)

            start_pose = np.random.uniform(low=low, high=high)
            x, y, z, yaw = start_pose

            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

            self._save_coordinate([start_pose])

    def undo_step(self) -> None:
        """
        Pushes the index of the current step back by 1.
        """
        from_i = self.view.steps.current_pose_index
        to_i = max(0, self.view.steps.current_pose_index - 1)
        self.view.steps.move_highlight(from_i, to_i)

        self.view.steps.current_pose_index = to_i
        self.view.steps.refresh_display()

        if coord := self.view.steps.get_current_pose():
            x, y, z, yaw = coord
            self.last_pose = coord
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def redo_step(self) -> None:
        """
        Pushes the index of the current step up by 1.
        """
        from_i = self.view.steps.current_pose_index
        to_i = min(
            len(self.view.steps.steps) - 1,
            self.view.steps.current_pose_index + 1,
        )
        self.view.steps.move_highlight(from_i, to_i)

        self.view.steps.current_pose_index = to_i
        self.view.steps.refresh_display()

        if coord := self.view.steps.get_current_pose():
            x, y, z, yaw = coord
            self.last_pose = coord
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def reset_demo(self) -> None:
        """
        Displays pop up to confirm resetting of demo data.
        """
        should_reset = messagebox.askyesno(
            title="Reset Demo",
            message="You will lose all data collected for this demo. Are you sure you want to reset the demo?",
            icon="warning",
        )

        if should_reset:
            self._reset_demo()
            self.update_graph()

    def update_graph(self, _event=None) -> None:
        """
        Event function for updating the graph displayed in the data section.
        """
        if self.csv_writer is None or self.project is None:
            self.view.data_section.clear_metrics("No collected data available.")
            return

        self.view.data_section.set_metric_order(
            self.project["tensor_spec"]["input_features"],
        )
        self.view.data_section.set_metrics_data(self.csv_writer.fetch_state_series())

    def show_edit_demo(self) -> None:
        """
        Allow editing of demo name and sampling rate.
        """

        # Show edit demo pop up
        if self.edit_demo_popup and self.edit_demo_popup.win.winfo_exists():
            self.edit_demo_popup.win.lift()
            self.edit_demo_popup.win.focus_force()
        else:
            self.edit_demo_popup = EditDemoPopup(
                self.view,
                self.demo,
                self._cancel_edit_demo,
                self._save_changes,
            )

    def _cancel_edit_demo(self) -> None:
        self.edit_demo_popup.win.destroy()
        self.edit_demo_popup = None

    def _save_changes(self) -> None:

        name = self.edit_demo_popup.name_entry.get()
        sampling_rate = float(self.edit_demo_popup.rate_entry.get())

        demo = self.demo.copy()

        demo["name"] = name
        demo["sampling_rate"] = sampling_rate

        try:
            # Attempt to edit demo.
            edit_demo(self.project, demo, original_demo_name=self.demo["name"])

            self.demo = demo

            self.view.header.demo_title.config(text=name)
            self.view.header.subtitle.config(
                text=(
                    f"Sampling rate: {self.demo['sampling_rate']} steps / sec | "
                    f"World: {self.project['world_file']}"
                ),
            )

            # Update CSV writer
            self.csv_writer.set_new_paths(self.project, self.demo)

            self._cancel_edit_demo()

        except FileExistsError:
            # Display warning label
            self.edit_demo_popup.warning_label.grid()

    def _reset_demo(self) -> None:
        """
        Resets all recorded data of a demo.
        """
        # Clear collected data
        self.csv_writer.clear_all_data()

        # Clear UI
        self.view.steps.clear()
        x, y, z, yaw = self.demo["start_position"]
        self.view.steps.add_step((x, y, z, yaw), True)

        # Enable preposition and random position buttons
        self.view.controls.preposition_button.config(state=tk.NORMAL)
        self.view.controls.random_position_button.config(
            state=(
                tk.NORMAL
                if self.project["random_spawn_space"]["enabled"]
                else tk.DISABLED
            ),
        )

        # Place sub in starting location
        self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def _schedule_next_sample(self) -> None:
        """
        Schedule the next sampling event.
        """
        if self.is_recording:

            self._record_step()

            self.view.after(self.delay, self._schedule_next_sample)

    def _record_step(self) -> None:
        """
        Record the state and action of step taken.
        """
        # Get pose data
        pose = self.get_pose_client.send_request()
        x, y, z, yaw = (pose.x, pose.y, pose.z, pose.yaw)
        last_x, last_y, last_z, last_yaw = self.last_pose
        dx, dy, dz, dyaw = (x - last_x, y - last_y, z - last_z, yaw - last_yaw)

        # Get input topic state data.
        data = json.loads(
            self.data_collector.get_snapshot(str(self.csv_writer.demo_dir_path)).data,
        )

        if data:
            feature_values = extract_selected_state_features(
                data,
                self.project["tensor_spec"]["input_features"],
            )

            # Record data into persistent CSV files.
            self.csv_writer.record(
                state=data,
                action={
                    "delta_x": dx,
                    "delta_y": dy,
                    "delta_z": dz,
                    "delta_yaw": dyaw,
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                },
            )

            # Display step in GUI
            self.view.steps.add_step((x, y, z, yaw))
            self.view.steps.canvas.yview_moveto(1.0)
            self.view.data_section.append_metric_point(
                {
                    metric_name: float(value)
                    for metric_name, value in feature_values.items()
                },
            )

            if self.view.controls.preposition_button["state"] == "normal":
                self.view.controls.preposition_button.config(state=tk.DISABLED)

    def _save_coordinate(self, coordinates: list[Coord4D]) -> None:
        """
        Save coordinate for the start position of the model.
        """
        coordinate = coordinates[0]

        if coordinate is None:
            return

        x, y, z, yaw = coordinate
        self.demo["start_position"] = (float(x), float(y), float(z), float(yaw))

        # Update config
        edit_demo(self.project, self.demo)

        # Display changes in UI
        self.view.steps.clear()
        self.view.steps.add_step((x, y, z, yaw), True)

        # Close out popups
        self.keyboard_controls_gui.hide()
        self.coordinate_popup = None

        # Pause simulation
        self.world_control_client.pause_simulation()

        # Enable play button
        self.view.controls.pause_button.config(state=tk.DISABLED)
        self.view.controls.play_button.config(state=tk.NORMAL)

    def _clean_components(self) -> None:
        self.view.header.destroy()
        self.view.content.destroy()
