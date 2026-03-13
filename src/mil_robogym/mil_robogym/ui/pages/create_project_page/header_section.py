from __future__ import annotations

import tkinter as tk
from typing import Callable


class HeaderSection:
    """Render the header breadcrumb row for the create-project page.

    Args:
        parent: Parent widget that hosts this section.
        on_home_click: Callback bound to the breadcrumb home label click.
    """

    def __init__(
        self,
        parent: tk.Widget,
        on_home_click: Callable[[object], None],
    ) -> None:
        """Initialize and place all widgets for the header section."""
        self.title_row = tk.Frame(parent, bg="#DADADA")
        self.title_row.grid(
            row=0,
            column=0,
            columnspan=6,
            sticky="w",
            padx=14,
            pady=(14, 8),
        )

        self.home_title = tk.Label(
            self.title_row,
            text="MIL Robogym >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
            cursor="hand2",
        )
        self.home_title.pack(side="left")
        self.home_title.bind("<Button-1>", on_home_click)

        self.page_title = tk.Label(
            self.title_row,
            text="Create Project",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        self.page_title.pack(side="left", padx=(6, 0))
