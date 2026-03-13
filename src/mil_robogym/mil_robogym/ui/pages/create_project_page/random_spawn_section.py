from __future__ import annotations

import tkinter as tk
from typing import Callable, cast

from mil_robogym.data_collection.types import Coord4D


class RandomSpawnSection:
    """Build and manage the random-spawn controls for project creation. This keeps behavior scoped to the current component.

    Args:
        parent: Parent widget that hosts the random-spawn widgets.
        on_state_change: Callback run when any random-spawn field changes.
        on_grab_from_sim: Callback run when the grab button is pressed.
    Returns:
        None.
    """

    def __init__(
        self,
        parent: tk.Widget,
        on_state_change: Callable[[], None],
        on_grab_from_sim: Callable[[], None],
    ) -> None:
        """Initialize section widgets and bind user interaction callbacks. This keeps behavior scoped to the current component.

        Args:
            parent: Parent widget that owns this section's controls.
            on_state_change: Callback used to notify external form-state updates.
            on_grab_from_sim: Callback used to start the simulation coordinate flow.
        Returns:
            None.
        """
        self._on_state_change = on_state_change
        self._on_grab_from_sim = on_grab_from_sim

        self.coordinate1: Coord4D | None = None
        self.coordinate2: Coord4D | None = None

        self.random_spawn_var = tk.BooleanVar(value=False)
        self.random_spawn_checkbox = tk.Checkbutton(
            parent,
            text="Random Spawn Space:",
            variable=self.random_spawn_var,
            command=self._on_toggle_random_spawn,
            bg="#DADADA",
            fg="black",
            activebackground="#DADADA",
            font=("Arial", 15, "bold"),
            highlightthickness=0,
            anchor="w",
            borderwidth=0,
        )
        self.random_spawn_checkbox.grid(
            row=4,
            column=0,
            columnspan=2,
            sticky="nsew",
            padx=(14, 8),
            pady=(6, 2),
        )

        self.coord1_label = tk.Label(
            parent,
            text="Coord 1:",
            bg="#DADADA",
            fg="#999999",
            font=("Arial", 15),
        )
        self.coord1_label.grid(
            row=5,
            column=0,
            columnspan=1,
            sticky="e",
            padx=(14, 8),
            pady=4,
        )

        self.coord1_var = tk.StringVar()
        self.coord1_var.trace_add("write", self._on_coord_change)
        self.coord1_entry = tk.Entry(
            parent,
            textvariable=self.coord1_var,
            font=("Arial", 15),
            state="disabled",
            disabledbackground="#ECECEC",
            disabledforeground="#666666",
        )
        self.coord1_entry.grid(
            row=5,
            column=1,
            columnspan=1,
            sticky="nsew",
            padx=(0, 8),
            pady=4,
            ipady=3,
        )

        self.coord2_label = tk.Label(
            parent,
            text="Coord 2:",
            bg="#DADADA",
            fg="#999999",
            font=("Arial", 15),
        )
        self.coord2_label.grid(
            row=5,
            column=2,
            columnspan=1,
            sticky="e",
            padx=(6, 8),
            pady=4,
        )

        self.coord2_var = tk.StringVar()
        self.coord2_var.trace_add("write", self._on_coord_change)
        self.coord2_entry = tk.Entry(
            parent,
            textvariable=self.coord2_var,
            font=("Arial", 15),
            state="disabled",
            disabledbackground="#ECECEC",
            disabledforeground="#666666",
        )
        self.coord2_entry.grid(
            row=5,
            column=3,
            columnspan=1,
            sticky="nsew",
            padx=(0, 8),
            pady=4,
            ipady=3,
        )

        self.grab_from_sim_button = tk.Button(
            parent,
            text="Grab from Sim",
            command=self._on_grab_from_sim_click,
            bg="#E5E5E5",
            activebackground="#D9D9D9",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 14),
            padx=10,
            pady=4,
            state="disabled",
            disabledforeground="#666666",
        )
        self.grab_from_sim_button.grid(
            row=5,
            column=5,
            sticky="nsew",
            padx=(0, 14),
            pady=4,
        )

    def _on_toggle_random_spawn(self) -> None:
        """Handle checkbox toggles by applying UI state and notifying listeners. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self._apply_widget_state()
        self._on_state_change()

    def _apply_widget_state(self) -> None:
        """Apply enabled or disabled styling based on the random-spawn toggle. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        enabled = self.random_spawn_var.get()
        state = "normal" if enabled else "disabled"
        label_color = "black" if enabled else "#999999"

        self.coord1_entry.configure(state=state)
        self.coord2_entry.configure(state=state)
        self.coord1_label.configure(fg=label_color)
        self.coord2_label.configure(fg=label_color)
        self.grab_from_sim_button.configure(
            state=state,
            fg="black" if enabled else "#666666",
        )

    def _on_coord_change(self, *_: object) -> None:
        """Handle coordinate text edits and notify external form-state listeners. This keeps behavior scoped to the current component.

        Args:
            *_: Unused Tk trace callback arguments.
        Returns:
            None.
        """
        self._on_state_change()

    def _on_grab_from_sim_click(self) -> None:
        """Handle click events for the simulation coordinate-grab button. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self._on_grab_from_sim()

    def set_collected_coords(self, c1: Coord4D | None, c2: Coord4D | None) -> None:
        """Update displayed coordinates from simulation-collected values. This keeps behavior scoped to the current component.

        Args:
            c1: First collected 4D coordinate or None if unavailable.
            c2: Second collected 4D coordinate or None if unavailable.
        Returns:
            None.
        """
        if c1 is None or c2 is None:
            return

        self.coordinate1 = c1
        self.coordinate2 = c2

        self.coord1_entry.delete(0, tk.END)
        self.coord1_entry.insert(0, f"{tuple(round(v, 1) for v in c1)}")

        self.coord2_entry.delete(0, tk.END)
        self.coord2_entry.insert(0, f"{tuple(round(v, 1) for v in c2)}")

        self._on_state_change()

    def parse_coord(self, value: str) -> Coord4D | None:
        """Parse a coordinate string into a typed 4D tuple. This keeps behavior scoped to the current component.

        Args:
            value: Raw text expected in the form "(x, y, z, yaw)".
        Returns:
            A Coord4D tuple when parsing succeeds, otherwise None.
        """
        cleaned = value.strip().strip("()")
        parts = [part.strip() for part in cleaned.split(",")]
        if len(parts) != 4:
            return None

        try:
            parsed = tuple(float(part) for part in parts)
        except ValueError:
            return None

        if len(parsed) != 4:
            return None
        return cast(Coord4D, parsed)

    def get_coords(self) -> tuple[Coord4D | None, Coord4D | None]:
        """Return both coordinates using collected values or parsed entry text. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            A tuple containing coord1 and coord2 values, each optional.
        """
        coord1 = self.coordinate1 or self.parse_coord(self.coord1_var.get())
        coord2 = self.coordinate2 or self.parse_coord(self.coord2_var.get())
        return coord1, coord2

    def is_random_spawn_enabled(self) -> bool:
        """Report whether random-spawn mode is currently enabled. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            True when random spawn is enabled, otherwise False.
        """
        return self.random_spawn_var.get()
