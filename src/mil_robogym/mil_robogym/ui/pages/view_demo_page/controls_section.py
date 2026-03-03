import tkinter as tk


class ControlsSection(tk.Frame):
    """
    Bottom control bar containing playback and editing controls.
    """

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, bg="#CFCFCF", height=60)
        self.grid_propagate(False)

        btns = tk.Frame(self, bg="#CFCFCF")
        btns.pack(pady=10)

        for label in ["Reset Demo", "Play/Continue", "Pause", "Undo", "Redo"]:
            tk.Button(btns, text=label, width=12).pack(side="left", padx=4)

        tk.Button(btns, text="Preposition", state="disabled", width=12).pack(
            side="left",
            padx=10,
        )
        tk.Button(btns, text="Rand. Pos.", state="disabled", width=12).pack(side="left")
