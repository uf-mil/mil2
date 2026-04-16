import tkinter as tk
from tkinter import ttk
from typing import Callable

from mil_robogym.data_collection.types import RoboGymDemoYaml
from mil_robogym.ui.components.tool_tip import ToolTip


class EditDemoPopup:
    def __init__(
        self,
        parent: tk.Frame,
        demo: RoboGymDemoYaml,
        on_cancel: Callable,
        on_save: Callable,
    ):
        self.parent = parent
        self.on_cancel = on_cancel

        self.win = tk.Toplevel(parent)
        self.win.title("Edit Demo")
        self.win.resizable(False, False)
        self.win.transient(parent)
        self.win.attributes("-topmost", True)

        container = ttk.Frame(self.win, padding=20)
        container.grid(row=0, column=0)

        # Demo Name Row
        ttk.Label(container, text="Demo Name:").grid(
            row=0,
            column=0,
            sticky="w",
            pady=5,
        )

        self.name_var = tk.StringVar(value=demo["name"])
        self.name_entry = ttk.Entry(container, textvariable=self.name_var, width=25)
        self.name_entry.grid(row=0, column=1, pady=5)
        self.name_entry.focus()

        self.warning_label = ttk.Label(container, text="⚠", foreground="red")
        self.warning_label.grid(row=0, column=2, padx=(5, 0))
        self.warning_label.grid_remove()  # hide initially
        self.tooltip = ToolTip(
            self.warning_label,
            "A demo with this name already exists.",
        )

        # Sampling Rate Row
        ttk.Label(container, text="Sampling Rate:").grid(
            row=1,
            column=0,
            sticky="w",
            pady=5,
        )

        self.rate_var = tk.StringVar(value=demo["sampling_rate"])

        # allow only number
        vcmd = (self.win.register(self._validate_number), "%P")
        self.rate_entry = ttk.Entry(
            container,
            textvariable=self.rate_var,
            validate="key",
            validatecommand=vcmd,
            width=10,
        )
        self.rate_entry.grid(row=1, column=1, sticky="w", pady=5)

        ttk.Label(container, text="steps / second").grid(row=1, column=2, padx=(8, 0))

        # Buttons Row
        button_frame = ttk.Frame(container)
        button_frame.grid(row=2, column=0, columnspan=3, pady=(15, 0))

        cancel_btn = ttk.Button(button_frame, text="Cancel", command=on_cancel)
        cancel_btn.pack(side="left", padx=5)

        create_btn = ttk.Button(button_frame, text="Save", command=on_save)
        create_btn.pack(side="left", padx=5)

        # Handle window close (same as cancel)
        self.win.protocol("WM_DELETE_WINDOW", on_cancel)

    def _validate_number(self, value):
        """Allow empty string or digits only."""
        if value == "":
            return True
        try:
            float(value)
            return True
        except ValueError:
            return False
