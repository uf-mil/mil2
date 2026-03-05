import tkinter as tk

from mil_robogym.data_collection.types import RoboGymProjectYaml

from .buttons.preposition import PrepositionButton
from .buttons.random_pose import RandomPoseButton


class ControlsSection(tk.Frame):
    """
    Bottom control bar containing playback and editing controls.
    """

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, bg="#CFCFCF", height=60)
        self.grid_propagate(False)

        self.parent = parent

        self.btns = tk.Frame(self, bg="#CFCFCF")
        self.btns.pack(pady=10)

        for label in ["Reset Demo", "Play/Continue", "Pause", "Undo", "Redo"]:
            tk.Button(self.btns, text=label, width=12).pack(side="left", padx=4)

    def enable_buttons(self, project: RoboGymProjectYaml) -> None:
        PrepositionButton(self.parent, self.btns, True)

        random_spawn_space = project["random_spawn_space"]
        RandomPoseButton(
            self.parent,
            self.btns,
            random_spawn_space["enabled"],
            random_spawn_space["coord1_4d"],
            random_spawn_space["coord2_4d"],
        )
