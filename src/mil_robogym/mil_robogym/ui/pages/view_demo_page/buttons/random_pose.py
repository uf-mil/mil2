import tkinter as tk

import numpy as np

from mil_robogym.clients.set_pose_client import SetPoseClient
from mil_robogym.data_collection.filesystem import edit_demo


class RandomPoseButton:
    """
    Button that assigns a random position and orientation to the sub.
    """

    def __init__(
        self,
        grand_parent: tk.Widget,
        parent: tk.Frame,
        isEnabled: bool = False,
        c1: np.ndarray | None = None,
        c2: np.ndarray | None = None,
    ) -> None:

        self.grand_parent = grand_parent

        self.isEnabled = isEnabled

        self.c1 = c1
        self.c2 = c2

        self.set_pose_client = SetPoseClient()

        tk.Button(
            parent,
            text="Rand. Pos.",
            state="active" if isEnabled else "disabled",
            width=12,
            command=self._on_click,
        ).pack(side="left")

    def _on_click(self) -> None:
        if self.isEnabled and self.c1 and self.c2:
            # Get random position and request new position to be set
            low = np.minimum(self.c1, self.c2)
            high = np.maximum(self.c1, self.c2)

            start_pose = np.random.uniform(low=low, high=high)
            x, y, z, yaw = start_pose

            self.set_pose_client.set_pose(x, y, z, yaw=yaw)

            project = self.grand_parent.project["robogym_project"]
            demo = self.grand_parent.demo
            demo["start_position"] = (float(x), float(y), float(z), float(yaw))

            # Update config yaml
            edit_demo(project, demo)

            # Update UI
            self.grand_parent.steps.clear()
            self.grand_parent.steps.add_step(
                f"Origin: ({x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f})",
            )
