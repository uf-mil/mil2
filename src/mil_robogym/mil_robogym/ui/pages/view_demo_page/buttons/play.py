import tkinter as tk
from typing import Callable

from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.set_pose_client import SetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient


class PlayButton:
    """
    Button that starts recording steps taken.
    """

    def __init__(
        self,
        grand_parent: tk.Widget,
        parent: tk.Frame,
        on_play: Callable,
        sampling_rate: float,
    ):

        self.grand_parent = grand_parent
        self.sampling_rate = sampling_rate
        self.on_play = on_play

        self.get_pose_client = GetPoseClient()
        self.set_pose_client = SetPoseClient()
        self.world_control_client = WorldControlClient()

        self.button = tk.Button(
            parent,
            text="Play",
            state="active",
            width=12,
            command=self._on_click,
        )
        self.button.pack(side="left")

    def _on_click(self) -> None:

        # Place sub on last position recorded
        coordinate = self.grand_parent.steps.last_pose
        if coordinate:
            x, y, z, yaw = coordinate
            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

        # Play the simulation
        self.world_control_client.play_simulation()

        # Start sampling asynchronously
        self.sampling_active = True
        self._schedule_next_sample()

        # Disable button
        self.button.config(state=tk.DISABLED)
        self.on_play()

    def _schedule_next_sample(self) -> None:
        """
        Schedule the next sampling event.
        """

        if not self.sampling_active:
            return

        self._record_step()

        # Convert seconds -> milliseconds
        delay = int((1 / self.sampling_rate) * 1000)

        self.grand_parent.after(delay, self._schedule_next_sample)

    def _record_step(self) -> None:

        pose = self.get_pose_client.send_request()
        x, y, z, yaw = (pose.x, pose.y, pose.z, pose.yaw)

        # TODO: Record state data and update CSVs

        # Display new step
        self.grand_parent.steps.add_step((x, y, z, yaw))
        self.grand_parent.steps.canvas.yview_moveto(1.0)
