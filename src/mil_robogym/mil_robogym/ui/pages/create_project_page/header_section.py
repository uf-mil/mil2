from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from typing import Callable


@dataclass
class HeaderSectionWidgets:
    title_row: tk.Frame
    home_title: tk.Label
    page_title: tk.Label


def build_header_section(
    parent: tk.Widget,
    on_home_click: Callable[[object], None],
) -> HeaderSectionWidgets:
    """Build the header breadcrumb row for the create-project page. It binds home navigation behavior. This keeps behavior scoped to the current component.

    Args:
        parent: Parent widget that hosts the header row.
        on_home_click: Callback bound to the breadcrumb home label click.
    Returns:
        HeaderSectionWidgets containing the created header widgets.
    """
    title_row = tk.Frame(parent, bg="#DADADA")
    title_row.grid(row=0, column=0, columnspan=6, sticky="w", padx=14, pady=(14, 8))

    home_title = tk.Label(
        title_row,
        text="MIL Robogym >",
        bg="#DADADA",
        fg="black",
        font=("Arial", 20, "bold"),
        anchor="w",
        cursor="hand2",
    )
    home_title.pack(side="left")
    home_title.bind("<Button-1>", on_home_click)

    page_title = tk.Label(
        title_row,
        text="Create Project",
        bg="#DADADA",
        fg="black",
        font=("Arial", 20, "bold"),
        anchor="w",
    )
    page_title.pack(side="left", padx=(6, 0))

    return HeaderSectionWidgets(
        title_row=title_row,
        home_title=home_title,
        page_title=page_title,
    )
