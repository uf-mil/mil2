import tkinter as tk
from typing import Any

import numpy as np

from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.set_pose_client import SetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.filesystem import edit_demo
from mil_robogym.data_collection.types import (
    Coord4D,
)
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI


class DemoViewController:
    """
    Class that handles logic for a demo.
    """

    def __init__(self, view: tk.Frame, app):
        self.view = view
        self.app = app

        self.project: dict[str, Any] | None = None
        self.demo: dict[str, Any] | None = None

        self.coordinate_popup = None

        self.is_recording = False
        self.last_pose = None
        self.delay = -1

        self.get_pose_client = GetPoseClient()
        self.set_pose_client = SetPoseClient()
        self.world_control_client = WorldControlClient()

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
                    tk.ACTIVE
                    if self.project["random_spawn_space"]["enabled"]
                    else tk.DISABLED
                ),
            )

            # TODO: Load steps in from CSV files

            self.view.steps.clear()
            x, y, z, yaw = self.demo["start_position"]
            self.view.steps.add_step((x, y, z, yaw), is_origin=True)

            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

    def navigate_to_home(self, _event: tk.Event | None = None) -> None:
        self._clean_components()
        self.app.show_page("start")

    def navigate_to_project(self, _event: tk.Event | None = None) -> None:
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

        # Enable keyboard controls
        self.keyboard_controls_gui.show()

        # Start gazebo simulation
        self.world_control_client.play_simulation()

        # Disable the play button and enable the pause button
        self.view.controls.pause_button.config(state=tk.ACTIVE)
        self.view.controls.play_button.config(state=tk.DISABLED)

        # Start sampling asynchronously
        self.is_recording = True
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
        self.view.controls.play_button.config(state=tk.ACTIVE)

    def preposition(self) -> None:
        """
        Enable keyboard controls to move the sub without recording steps.
        """
        # Display coordinate popup
        if self.coordinate_popup and self.popup.win.winfo_exists():
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

        # TODO: Get input topic state data.

        # TODO: Record data into persistent CSV files.

        # Display step in GUI
        self.view.steps.add_step((x, y, z, yaw))
        self.view.steps.canvas.yview_moveto(1.0)

    def _save_coordinate(self, coordinates: list[Coord4D]) -> None:
        """
        Save coordinate for the start position of the model.
        """
        coordinate = coordinates[0]
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
        self.view.controls.play_button.config(state=tk.ACTIVE)

    def _clean_components(self) -> None:
        self.view.header.destroy()
        self.view.content.destroy()
