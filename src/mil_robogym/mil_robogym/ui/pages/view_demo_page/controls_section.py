import tkinter as tk


class ControlsSection(tk.Frame):
    """
    Bottom control bar containing playback and editing controls.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#CFCFCF", height=60)
        self.grid_propagate(False)

        self.is_recording = False

        self.parent = parent
        self.keyboard_controls_gui = None

        self.btns = tk.Frame(self, bg="#CFCFCF")
        self.btns.pack(pady=10)

        self.play_button = tk.Button(
            self.btns,
            text="Play",
            state="active",
            width=12,
            command=controller.start_recording,
        )
        self.play_button.pack(side="left", padx=4)

        self.pause_button = tk.Button(
            self.btns,
            text="Pause",
            state="disabled",
            width=12,
            command=controller.pause_recording,
        )
        self.pause_button.pack(side="left", padx=4)

        self.preposition_button = tk.Button(
            self.btns,
            text="Preposition",
            state="active",
            width=12,
            command=controller.preposition,
        )
        self.preposition_button.pack(side="left", padx=4)

        self.random_position_button = tk.Button(
            self.btns,
            text="Rand. Pos.",
            state="active",
            width=12,
            command=controller.set_random_origin,
        )
        self.random_position_button.pack(side="left", padx=4)
