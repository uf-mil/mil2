import tkinter as tk
from typing import Callable

from mil_robogym.clients.world_control_client import WorldControlClient


class PauseButton:
    """
    BUtton that stops recording the steps taken.
    """

    def __init__(
        self,
        grand_parent: tk.Widget,
        parent: tk.Frame,
        on_pause: Callable,
    ):
        self.grand_parent = grand_parent
        self.on_pause = on_pause

        self.world_control_client = WorldControlClient()

        self.button = tk.Button(
            parent,
            text="Pause",
            state="disabled",
            width=12,
            command=self._on_click,
        )
        self.button.pack(side="left")

    def enable(self):
        """
        Enable the pause button.
        """
        self.button.config(state=tk.ACTIVE)

    def _on_click(self) -> None:

        # Pause the simulation
        self.world_control_client.pause_simulation()

        # Disable keyboard controls
        self.button.config(state=tk.DISABLED)
        self.on_pause()
