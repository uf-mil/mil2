import tkinter as tk

from mil_robogym.data_collection.types import RoboGymProjectYaml
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI

from .buttons.pause import PauseButton
from .buttons.play import PlayButton
from .buttons.preposition import PrepositionButton
from .buttons.random_pose import RandomPoseButton


class ControlsSection(tk.Frame):
    """
    Bottom control bar containing playback and editing controls.
    """

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, bg="#CFCFCF", height=60)
        self.grid_propagate(False)

        self.is_recording = False

        self.parent = parent
        self.keyboard_controls_gui = None

        self.btns = tk.Frame(self, bg="#CFCFCF")
        self.btns.pack(pady=10)

        self.parent.bind("<space>", self._on_pause)

    def enable_buttons(self, project: RoboGymProjectYaml) -> None:
        self.play_button = PlayButton(
            self.parent,
            self.btns,
            self._on_play,
            self.parent.demo["sampling_rate"],
        )

        self.pause_button = PauseButton(self.parent, self.btns, self._on_pause)

        self.preposition_button = PrepositionButton(self.parent, self.btns, True)

        random_spawn_space = project["random_spawn_space"]
        self.random_spawn_space_button = RandomPoseButton(
            self.parent,
            self.btns,
            random_spawn_space["enabled"],
            random_spawn_space["coord1_4d"],
            random_spawn_space["coord2_4d"],
        )

        for label in ["Reset Demo", "Undo", "Redo"]:
            tk.Button(self.btns, text=label, width=12).pack(side="left", padx=4)

    def _on_pause(self) -> None:

        if self.is_recording:
            self.is_recording = False

            if self.keyboard_controls_gui:
                self.keyboard_controls_gui.hide()

            self._on_close_of_keyboard_controls()

    def _on_play(self) -> None:

        if not self.is_recording:
            # Start up keyboard controls
            self.keyboard_controls_gui = (
                self.keyboard_controls_gui
                or KeyboardControlsGUI(
                    self.parent,
                    self._on_close_of_keyboard_controls,
                )
            )
            self.keyboard_controls_gui.show()

            self.pause_button.enable()
            self.is_recording = True

    def _on_close_of_keyboard_controls(self) -> None:
        """
        Stop sampling when keyboard controls close.
        """
        self.is_recording = False
        self.play_button.sampling_active = False
        self.play_button.world_control_client.pause_simulation()
        self.play_button.button.config(state=tk.ACTIVE)
        self.pause_button.button.config(state=tk.DISABLED)
