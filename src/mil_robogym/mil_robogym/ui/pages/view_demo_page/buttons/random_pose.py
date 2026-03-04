import tkinter as tk

import numpy as np

from mil_robogym.clients.set_pose_client import SetPoseClient


class RandomPoseButton:
    """
    Button that assigns a random position and orientation to the sub.
    """

    def __init__(
        self,
        parent: tk.Frame,
        isEnabled: bool = False,
        c1: np.ndarray | None = None,
        c2: np.ndarray | None = None,
    ) -> None:

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

            low = np.minimum(self.c1, self.c2)
            high = np.maximum(self.c1, self.c2)

            x, y, z, yaw = np.random.uniform(low=low, high=high)

            self.set_pose_client.set_pose(x, y, z, yaw=yaw)
