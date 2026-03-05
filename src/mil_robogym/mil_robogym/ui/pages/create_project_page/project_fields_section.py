from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from typing import Callable


@dataclass
class ProjectFieldsSectionWidgets:
    project_name_var: tk.StringVar
    world_file_var: tk.StringVar
    model_name_var: tk.StringVar
    project_name_entry: tk.Entry
    world_file_entry: tk.Entry
    model_name_entry: tk.Entry


def build_project_fields_section(
    parent: tk.Widget,
    world_default: str,
    on_project_name_change: Callable[[], None],
) -> ProjectFieldsSectionWidgets:
    """Build the project, world, and model field widgets for the page. It wires project-name changes to form validation. This keeps behavior scoped to the current component.

    Args:
        parent: Parent widget that hosts the field controls.
        world_default: Default world-file string used to initialize the field.
        on_project_name_change: Callback fired when the project-name value changes.
    Returns:
        ProjectFieldsSectionWidgets with the created variables and entry widgets.
    """
    project_name_label = tk.Label(
        parent,
        text="Project Name:",
        bg="#DADADA",
        fg="black",
        font=("Arial", 15),
        anchor="w",
    )
    project_name_label.grid(row=1, column=0, sticky="w", padx=(14, 8), pady=5)

    project_name_var = tk.StringVar()
    project_name_var.trace_add("write", lambda *_: on_project_name_change())
    project_name_entry = tk.Entry(
        parent,
        textvariable=project_name_var,
        font=("Arial", 15),
    )
    project_name_entry.grid(
        row=1,
        column=1,
        columnspan=5,
        sticky="nsew",
        padx=(0, 14),
        pady=5,
        ipady=3,
    )

    world_file_label = tk.Label(
        parent,
        text="World File:",
        bg="#DADADA",
        fg="black",
        font=("Arial", 15),
        anchor="w",
    )
    world_file_label.grid(
        row=2,
        column=0,
        sticky="w",
        padx=(14, 8),
        pady=5,
    )
    world_file_var = tk.StringVar(value=world_default)
    world_file_entry = tk.Entry(
        parent,
        textvariable=world_file_var,
        font=("Arial", 15),
        state=tk.DISABLED,
    )
    world_file_entry.grid(
        row=2,
        column=1,
        columnspan=4,
        sticky="nsew",
        padx=(0, 8),
        pady=5,
        ipady=3,
    )

    model_name_label = tk.Label(
        parent,
        text="Model Name:",
        bg="#DADADA",
        fg="black",
        font=("Arial", 15),
        anchor="w",
    )
    model_name_label.grid(
        row=3,
        column=0,
        sticky="w",
        padx=(14, 8),
        pady=5,
    )
    model_name_var = tk.StringVar(value="sub9")
    model_name_entry = tk.Entry(
        parent,
        textvariable=model_name_var,
        font=("Arial", 15),
        state=tk.DISABLED,
    )
    model_name_entry.grid(
        row=3,
        column=1,
        columnspan=3,
        sticky="nsew",
        padx=(0, 8),
        pady=5,
        ipady=3,
    )

    return ProjectFieldsSectionWidgets(
        project_name_var=project_name_var,
        world_file_var=world_file_var,
        model_name_var=model_name_var,
        project_name_entry=project_name_entry,
        world_file_entry=world_file_entry,
        model_name_entry=model_name_entry,
    )
