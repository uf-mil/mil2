import json
import time
import tkinter as tk
from contextlib import suppress
from tkinter import messagebox
from typing import Any

import numpy as np

from mil_robogym.clients.data_collector_client import DataCollectorClient
from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.set_pose_client import SetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.filesystem import edit_demo
from mil_robogym.data_collection.topic_readiness import (
    DEFAULT_TOPIC_READINESS_TIMEOUT_S,
    ensure_project_input_topics_ready,
)
from mil_robogym.data_collection.types import (
    Coord4D,
    RoboGymDemoYaml,
    RoboGymProjectYaml,
)
from mil_robogym.data_collection.utils import extract_selected_state_features
from mil_robogym.data_collection.writers.csv_writer import AsyncCSVWriter
from mil_robogym.ui.components.create_demo_popup import CreateDemoPopup
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI

from .edit_demo_popup import EditDemoPopup

SEQUENCE_PLAYBACK_DELAY_MS = 250


def _graph_sample_index_for_step_index(step_index: int | None) -> int | None:
    if step_index is None or step_index <= 0:
        return None
    return step_index - 1


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
        self.create_demo_popup = None

        self.is_recording = False
        self.last_pose = None
        self.delay = -1
        self.sample_period_s = 0.0
        self._countdown_after_id: str | None = None
        self._next_sample_deadline_s: float | None = None
        self._sequence_playback_after_id: str | None = None
        self._sequence_playback_index: int | None = None
        self._has_sequence_playback_hold = False

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
        self._stop_sequence_playback()
        self._clear_selected_step()

        self.raw_project = project
        self.project = project.get("robogym_project", {})
        self.demo = demo.get("robogym_demo", {})

        if self.project and self.demo:

            self._set_sampling_interval(float(self.demo["sampling_rate"]))

            # Create and start services
            self.csv_writer = AsyncCSVWriter(self.project, self.demo)

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
            self._refresh_sequence_playback_ui()

    def navigate_to_home(self, _event: tk.Event | None = None) -> None:
        self._close_current_demo()
        self.app.show_page("start")

    def navigate_to_project(self, _event: tk.Event | None = None) -> None:
        self._close_current_demo()
        self.app.show_page("view_project", project=self.raw_project)

    def move_model_to(self, coordinate: Coord4D) -> None:
        """
        Move the model to the provided position.
        """
        x, y, z, yaw = coordinate
        self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def select_step(self, index: int) -> None:
        self._stop_sequence_playback()
        self._select_step(index)

    def toggle_recording(self) -> None:
        if self.is_recording:
            self.pause_recording()
            return

        if str(self.view.controls.play_button.cget("state")) != str(tk.DISABLED):
            self.start_recording()

    def toggle_sequence_playback(self) -> None:
        if self._is_sequence_playback_running():
            self._stop_sequence_playback()
            return
        self.play_recorded_sequence()

    def start_recording(self) -> None:
        """
        Starts recording steps.
        """
        if str(self.view.controls.play_button.cget("state")) == str(tk.DISABLED):
            return

        self._clear_sample_countdown()
        self._stop_sequence_playback()
        self._clear_selected_step()
        self.view.controls.play_button.config(state=tk.DISABLED)
        self.view.steps.set_status_message("Checking input topics and publishers...")
        self.view.update_idletasks()

        if not self._ensure_recording_topics_ready():
            self.view.steps.set_status_message("")
            self.view.controls.play_button.config(state=tk.NORMAL)
            return

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
        self.view.steps.set_status_message("")
        self._refresh_sequence_playback_ui()

        # Delay sampling if mid session
        if len(self.view.steps.steps) > 1:
            self._schedule_sample_countdown(self.sample_period_s)
            self.view.after(self.delay, self._schedule_next_sample)
        else:
            self._schedule_next_sample()

    def _ensure_recording_topics_ready(self) -> bool:
        if not self.project:
            messagebox.showerror(
                "Play Demo",
                "Recording unavailable: no project is loaded.",
            )
            return False

        try:
            ensure_project_input_topics_ready(
                self.project,
                timeout_s=DEFAULT_TOPIC_READINESS_TIMEOUT_S,
                operation="start or continue recording",
                start_simulation=True,
            )
            self.data_collector.ensure_subscriptions(
                list(self.project["input_topics"].keys()),
                operation="prepare data collector subscriptions for recording",
            )
        except RuntimeError as exc:
            messagebox.showerror("Play Demo", str(exc))
            return False

        return True

    def pause_recording(self) -> None:
        """
        Pauses the recording of steps.
        """
        if str(self.view.controls.pause_button.cget("state")) == str(tk.DISABLED):
            return

        self._clear_sample_countdown()

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
        self._refresh_sequence_playback_ui()

    def preposition(self) -> None:
        """
        Enable keyboard controls to move the sub without recording steps.
        """
        if str(self.view.controls.preposition_button.cget("state")) == str(tk.DISABLED):
            return

        self._stop_sequence_playback()

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
        if str(self.view.controls.random_position_button.cget("state")) == str(
            tk.DISABLED,
        ):
            return

        self._stop_sequence_playback()

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
        if str(self.view.controls.undo_button.cget("state")) == str(tk.DISABLED):
            return

        self._stop_sequence_playback()

        from_i = self.view.steps.current_pose_index
        to_i = max(0, self.view.steps.current_pose_index - 1)
        self.view.steps.move_highlight(from_i, to_i)

        self.view.steps.current_pose_index = to_i
        self.view.steps.refresh_display()
        self._sync_selected_step_to_visible_range()
        self._refresh_sequence_playback_ui()

        if coord := self.view.steps.get_current_pose():
            x, y, z, yaw = coord
            self.last_pose = coord
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def redo_step(self) -> None:
        """
        Pushes the index of the current step up by 1.
        """
        if str(self.view.controls.redo_button.cget("state")) == str(tk.DISABLED):
            return

        self._stop_sequence_playback()

        from_i = self.view.steps.current_pose_index
        to_i = min(
            len(self.view.steps.steps) - 1,
            self.view.steps.current_pose_index + 1,
        )
        self.view.steps.move_highlight(from_i, to_i)

        self.view.steps.current_pose_index = to_i
        self.view.steps.refresh_display()
        self._sync_selected_step_to_visible_range()
        self._refresh_sequence_playback_ui()

        if coord := self.view.steps.get_current_pose():
            x, y, z, yaw = coord
            self.last_pose = coord
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def reset_demo(self) -> None:
        """
        Displays pop up to confirm resetting of demo data.
        """
        if str(self.view.controls.reset_demo_button.cget("state")) == str(tk.DISABLED):
            return

        self._stop_sequence_playback()

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
        self._update_selected_graph_marker()

    def play_recorded_sequence(self) -> None:
        """
        Play back the visible sequence of recorded steps.
        """
        if self.is_recording or self._is_sequence_playback_running():
            return

        end_index = self._visible_sequence_end_index()
        if end_index < 1:
            return

        start_index = (
            self.view.steps.selected_index
            if 0 <= self.view.steps.selected_index <= end_index
            else 0
        )

        self._acquire_sequence_playback_hold()
        self._sequence_playback_index = start_index
        self._select_step(start_index)

        if start_index >= end_index:
            self._stop_sequence_playback()
            return

        self._sequence_playback_after_id = self.view.after(
            SEQUENCE_PLAYBACK_DELAY_MS,
            self._advance_sequence_playback,
        )
        self._refresh_sequence_playback_ui()

    def show_edit_demo(self) -> None:
        """
        Allow editing of demo name and sampling rate.
        """
        if self.is_recording or self._has_active_popup(exclude="edit"):
            return

        self._stop_sequence_playback()

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

    def show_create_demo(self) -> None:
        """Open the quick-create popup for a fresh demo in the same project."""
        if self.is_recording or self._has_active_popup(exclude="create"):
            return

        self._stop_sequence_playback()

        if self.create_demo_popup and self.create_demo_popup.win.winfo_exists():
            self.create_demo_popup.win.lift()
            self.create_demo_popup.win.focus_force()
            return

        self.create_demo_popup = CreateDemoPopup(
            self.view,
            project=self.project,
            on_created=self._open_created_demo,
            on_cancel=self._cancel_create_demo,
            default_sampling_rate=float(self.demo["sampling_rate"]),
            default_start_position=tuple(self.demo["start_position"]),
        )

    def _cancel_edit_demo(self) -> None:
        self.edit_demo_popup.win.destroy()
        self.edit_demo_popup = None

    def _cancel_create_demo(self) -> None:
        self.create_demo_popup = None

    def _open_created_demo(self, demo_name: str, demo_cfg: dict[str, Any]) -> None:
        self.create_demo_popup = None
        self._close_current_demo()
        self.app.show_page(
            "view_demo",
            project=self.raw_project,
            demo_name=demo_name,
            demo=demo_cfg,
        )

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
            self._set_sampling_interval(sampling_rate)

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
            self._refresh_sequence_playback_ui()

        except FileExistsError:
            # Display warning label
            self.edit_demo_popup.warning_label.grid()

    def _reset_demo(self) -> None:
        """
        Resets all recorded data of a demo.
        """
        self._clear_sample_countdown()

        # Clear collected data
        self.csv_writer.clear_all_data()

        # Clear UI
        self.view.steps.clear()
        x, y, z, yaw = self.demo["start_position"]
        self.view.steps.add_step((x, y, z, yaw), True)
        self._clear_selected_step()
        self._refresh_sequence_playback_ui()

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
            self._schedule_sample_countdown(self.sample_period_s)
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
        data = json.loads(self.data_collector.get_snapshot().data)

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
                    if isinstance(value, (int, float))
                },
            )

            if self.view.controls.preposition_button["state"] == "normal":
                self.view.controls.preposition_button.config(state=tk.DISABLED)

            self._refresh_sequence_playback_ui()

    def _save_coordinate(self, coordinates: list[Coord4D]) -> None:
        """
        Save coordinate for the start position of the model.
        """
        self._clear_sample_countdown()
        self._stop_sequence_playback()

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
        self._clear_selected_step()
        self._refresh_sequence_playback_ui()

        # Close out popups
        self.keyboard_controls_gui.hide()
        self.coordinate_popup = None

        # Pause simulation
        self.world_control_client.pause_simulation()

        # Enable play button
        self.view.controls.pause_button.config(state=tk.DISABLED)
        self.view.controls.play_button.config(state=tk.NORMAL)

    def _clean_components(self) -> None:
        self._clear_sample_countdown()
        self._stop_sequence_playback()
        self._destroy_popup("create")
        self._destroy_popup("edit")
        self._destroy_popup("coordinate")
        if hasattr(self.view, "cleanup"):
            self.view.cleanup()
        self.view.header.destroy()
        self.view.content.destroy()
        self.view.controls.destroy()

    def _set_sampling_interval(self, sampling_rate: float) -> None:
        self.sample_period_s = 1.0 / sampling_rate
        self.delay = int(self.sample_period_s * 1000)

    def _close_current_demo(self) -> None:
        self._clear_sample_countdown()
        self._stop_sequence_playback()

        if (
            self.csv_writer is not None
            and self.view.steps.current_pose_index != len(self.view.steps.steps) - 1
        ):
            self.csv_writer.clear_all_data(self.view.steps.current_pose_index)

        if self.csv_writer is not None:
            self.csv_writer.close()

        self._clean_components()

    def _countdown_enabled(self) -> bool:
        return self.sample_period_s > 1.0

    def _schedule_sample_countdown(self, delay_s: float) -> None:
        if not self.is_recording or not self._countdown_enabled():
            self._clear_sample_countdown()
            return

        self._next_sample_deadline_s = time.monotonic() + delay_s
        self._update_sample_countdown()

    def _update_sample_countdown(self) -> None:
        self._countdown_after_id = None

        if (
            not self.is_recording
            or not self._countdown_enabled()
            or self._next_sample_deadline_s is None
        ):
            self._clear_sample_countdown()
            return

        remaining_s = self._next_sample_deadline_s - time.monotonic()
        if remaining_s <= 0:
            self.view.steps.set_countdown_message("Recording next step...")
        else:
            self.view.steps.set_countdown_message(
                f"Next step in {remaining_s:.1f}s",
            )

        self._countdown_after_id = self.view.after(
            100,
            self._update_sample_countdown,
        )

    def _clear_sample_countdown(self) -> None:
        if self._countdown_after_id is not None:
            with suppress(Exception):
                self.view.after_cancel(self._countdown_after_id)
        self._countdown_after_id = None
        self._next_sample_deadline_s = None
        if hasattr(self.view, "steps"):
            self.view.steps.set_countdown_message("")

    def _select_step(self, index: int) -> None:
        end_index = self._visible_sequence_end_index()
        if not (0 <= index <= end_index):
            return

        coordinate = self.view.steps.get_step_coordinate(index)
        if coordinate is None:
            return

        self.move_model_to(coordinate)
        self.view.steps.set_selected_step(index)
        self._update_selected_graph_marker()

    def _clear_selected_step(self) -> None:
        if hasattr(self.view, "steps"):
            self.view.steps.clear_selected_step()
        if hasattr(self.view, "data_section"):
            self.view.data_section.set_selected_sample_index(None)

    def _update_selected_graph_marker(self) -> None:
        selected_step_index = (
            self.view.steps.selected_index if hasattr(self.view, "steps") else -1
        )
        self.view.data_section.set_selected_sample_index(
            _graph_sample_index_for_step_index(
                selected_step_index if selected_step_index >= 0 else None,
            ),
        )

    def _visible_sequence_end_index(self) -> int:
        return min(
            self.view.steps.current_pose_index,
            len(self.view.steps.steps) - 1,
        )

    def _has_playable_sequence(self) -> bool:
        return self._visible_sequence_end_index() >= 1

    def _sync_selected_step_to_visible_range(self) -> None:
        if self.view.steps.selected_index <= self._visible_sequence_end_index():
            self._update_selected_graph_marker()
            return
        self._clear_selected_step()

    def _refresh_sequence_playback_ui(self) -> None:
        self._sync_selected_step_to_visible_range()
        is_playing = self._is_sequence_playback_running()
        self.view.steps.set_play_sequence_button_state(
            enabled=self._has_playable_sequence() and not self.is_recording,
            is_playing=is_playing,
        )

    def _is_sequence_playback_running(self) -> bool:
        return (
            self._sequence_playback_index is not None
            or self._sequence_playback_after_id is not None
        )

    def _advance_sequence_playback(self) -> None:
        self._sequence_playback_after_id = None

        end_index = self._visible_sequence_end_index()
        if self._sequence_playback_index is None:
            self._stop_sequence_playback()
            return

        next_index = self._sequence_playback_index + 1
        if next_index > end_index:
            self._stop_sequence_playback()
            return

        self._sequence_playback_index = next_index
        self._select_step(next_index)

        if next_index >= end_index:
            self._stop_sequence_playback()
            return

        self._sequence_playback_after_id = self.view.after(
            SEQUENCE_PLAYBACK_DELAY_MS,
            self._advance_sequence_playback,
        )

    def _acquire_sequence_playback_hold(self) -> None:
        if self._has_sequence_playback_hold:
            return
        self.world_control_client.acquire_simulation_hold()
        self._has_sequence_playback_hold = True

    def _release_sequence_playback_hold(self) -> None:
        if not self._has_sequence_playback_hold:
            return
        self.world_control_client.release_simulation_hold()
        self._has_sequence_playback_hold = False

    def _stop_sequence_playback(self) -> None:
        if self._sequence_playback_after_id is not None:
            with suppress(Exception):
                self.view.after_cancel(self._sequence_playback_after_id)
        self._sequence_playback_after_id = None
        self._sequence_playback_index = None
        self._release_sequence_playback_hold()
        if hasattr(self.view, "steps"):
            self._refresh_sequence_playback_ui()

    def _popup_is_open(self, popup) -> bool:
        return bool(
            popup is not None
            and getattr(popup, "win", None) is not None
            and popup.win.winfo_exists(),
        )

    def has_active_popup(self) -> bool:
        return self._has_active_popup()

    def _has_active_popup(self, *, exclude: str | None = None) -> bool:
        popup_map = {
            "coordinate": self.coordinate_popup,
            "edit": self.edit_demo_popup,
            "create": self.create_demo_popup,
        }
        return any(
            name != exclude and self._popup_is_open(popup)
            for name, popup in popup_map.items()
        )

    def _destroy_popup(self, popup_name: str) -> None:
        popup_map = {
            "coordinate": self.coordinate_popup,
            "edit": self.edit_demo_popup,
            "create": self.create_demo_popup,
        }
        popup = popup_map.get(popup_name)
        if not self._popup_is_open(popup):
            return

        with suppress(Exception):
            popup.win.destroy()

        if popup_name == "coordinate":
            self.coordinate_popup = None
        elif popup_name == "edit":
            self.edit_demo_popup = None
        elif popup_name == "create":
            self.create_demo_popup = None
