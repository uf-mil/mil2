import tkinter as tk
from tkinter import ttk
from typing import Callable

from mil_robogym.data_collection.filesystem import create_demo_folder
from mil_robogym.data_collection.get_all_project_config import find_projects_dir
from mil_robogym.data_collection.utils import to_lower_snake_case


class CreateDemoPopup:
    def __init__(self, parent: tk.Frame, on_cancel: Callable[[], None] | None = None):
        """
        parent      : root Tk window
        on_cancel() : function that executes when the pop up is toggled away.
        """
        self.parent = parent
        self.on_cancel = on_cancel

        self.win = tk.Toplevel(parent)
        self.win.title("Record Demo")
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

        self.name_var = tk.StringVar(value="Demo #")
        self.name_entry = ttk.Entry(container, textvariable=self.name_var, width=25)
        self.name_entry.grid(row=0, column=1, pady=5)
        self.name_entry.focus()

        # Sampling Rate Row
        ttk.Label(container, text="Sampling Rate:").grid(
            row=1,
            column=0,
            sticky="w",
            pady=5,
        )

        self.rate_var = tk.StringVar(value="10")

        # allow only integers
        vcmd = (self.win.register(self._validate_int), "%P")
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

        cancel_btn = ttk.Button(button_frame, text="Cancel", command=self._cancel)
        cancel_btn.pack(side="left", padx=5)

        create_btn = ttk.Button(button_frame, text="Create Demo", command=self._create)
        create_btn.pack(side="left", padx=5)

        # Handle window close (same as cancel)
        self.win.protocol("WM_DELETE_WINDOW", self._cancel)

    def _validate_int(self, value):
        """Allow empty string or digits only."""
        return value.isdigit() or value == ""

    def _create(self):
        """User clicked 'Create Demo', a demo folder is created, and user is brought to demo page."""
        name = self.name_var.get().strip()
        rate_text = self.rate_var.get().strip()

        sampling_rate = int(rate_text) if rate_text else None

        _path, cfg = create_demo_folder(
            find_projects_dir() / to_lower_snake_case(self.parent.project_name),
            demo_name=name,
            sampling_rate=sampling_rate,
        )

        self.win.destroy()

        self.parent.controller.show_page(
            "view_demo",
            project=self.parent.project,
            demo_name=name,
            demo=cfg,
        )

    def _cancel(self):
        """User cancelled and pop up is removed."""
        if self.on_cancel:
            self.on_cancel()

        self.win.destroy()
