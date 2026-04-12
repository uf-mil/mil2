import tkinter as tk
from tkinter import ttk
from typing import Callable

from mil_robogym.data_collection.filesystem import create_demo_folder
from mil_robogym.data_collection.get_all_demo_config import get_all_demo_config
from mil_robogym.data_collection.types import Coord4D, RoboGymProjectYaml
from mil_robogym.ui.components.tool_tip import ToolTip


def _default_demo_name(project: RoboGymProjectYaml) -> str:
    try:
        num_existing_demos = len(get_all_demo_config(project["name"]))
    except (FileNotFoundError, ValueError, KeyError, TypeError):
        num_existing_demos = 0

    return f"Demo {num_existing_demos + 1}"


class CreateDemoPopup:
    def __init__(
        self,
        parent: tk.Frame,
        *,
        project: RoboGymProjectYaml,
        on_created: Callable[[str, dict], None],
        on_cancel: Callable[[], None] | None = None,
        default_name: str | None = None,
        default_sampling_rate: float = 10.0,
        default_start_position: Coord4D | None = None,
    ):
        """
        parent      : root Tk window
        on_cancel() : function that executes when the pop up is toggled away.
        """
        self.parent = parent
        self.project = project
        self.on_created = on_created
        self.on_cancel = on_cancel
        self.default_start_position = default_start_position

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

        self.name_var = tk.StringVar(value=default_name or _default_demo_name(project))
        self.name_entry = ttk.Entry(container, textvariable=self.name_var, width=25)
        self.name_entry.grid(row=0, column=1, pady=5)
        self.name_entry.focus()

        self.warning_label = ttk.Label(container, text="⚠", foreground="red")
        self.warning_label.grid(row=0, column=2, padx=(5, 0))
        self.warning_label.grid_remove()
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

        self.rate_var = tk.StringVar(value=str(default_sampling_rate))

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

        cancel_btn = ttk.Button(button_frame, text="Cancel", command=self._cancel)
        cancel_btn.pack(side="left", padx=5)

        create_btn = ttk.Button(button_frame, text="Create Demo", command=self._create)
        create_btn.pack(side="left", padx=5)

        # Handle window close (same as cancel)
        self.win.protocol("WM_DELETE_WINDOW", self._cancel)
        self.win.bind("<Return>", lambda _event: self._create())
        self.win.bind("<Escape>", lambda _event: self._cancel())

    def _validate_number(self, value):
        """Allow empty string or digits only."""
        if value == "":
            return True
        try:
            float(value)
            return True
        except ValueError:
            return False

    def _create(self):
        """User clicked 'Create Demo', a demo folder is created, and user is brought to demo page."""
        name = self.name_var.get().strip()
        rate_text = self.rate_var.get().strip()
        self.warning_label.grid_remove()

        sampling_rate = float(rate_text) if rate_text else None

        try:
            _path, cfg = create_demo_folder(
                self.project,
                name=name,
                sampling_rate=sampling_rate,
                start_position=self.default_start_position,
            )
        except FileExistsError:
            self.warning_label.grid()
            return

        self.win.destroy()
        self.on_created(name, cfg)

    def _cancel(self):
        """User cancelled and pop up is removed."""
        if self.on_cancel:
            self.on_cancel()

        self.win.destroy()
