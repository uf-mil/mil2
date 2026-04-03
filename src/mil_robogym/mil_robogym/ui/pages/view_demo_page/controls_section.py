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
            state="normal",
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

        self.undo_button = tk.Button(
            self.btns,
            text="Undo",
            state="normal",
            width=12,
            command=controller.undo_step,
        )
        self.undo_button.pack(side="left", padx=4)

        self.redo_button = tk.Button(
            self.btns,
            text="Redo",
            state="normal",
            width=12,
            command=controller.redo_step,
        )
        self.redo_button.pack(side="left", padx=4)

        self.reset_demo_button = tk.Button(
            self.btns,
            text="Reset Demo",
            state="normal",
            width=12,
            command=controller.reset_demo,
        )
        self.reset_demo_button.pack(side="left", padx=4)

        self.preposition_button = tk.Button(
            self.btns,
            text="Preposition",
            state="normal",
            width=12,
            command=controller.preposition,
        )
        self.preposition_button.pack(side="left", padx=4)

        self.random_position_button = tk.Button(
            self.btns,
            text="Rand. Pos.",
            state="normal",
            width=12,
            command=controller.set_random_origin,
        )
        self.random_position_button.pack(side="left", padx=4)
