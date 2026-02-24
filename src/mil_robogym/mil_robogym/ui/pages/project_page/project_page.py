import tkinter as tk
from typing import Any, Mapping


class ProjectPage(tk.Frame):
    """
    UI page for displaying information about a single RoboGym project.

    This frame is responsible for presenting high-level project metadata
    and updating its view when the active project context changes.
    """

    def __init__(
        self,
        parent: tk.Widget,
        controller: Any | None = None,
    ) -> None:
        """
        Initialize the ProjectPage frame.

        :param parent: Parent Tkinter widget.
        :param controller: Optional application controller used for navigation or shared state.
        """

        super().__init__(parent, bg="#DADADA")
        self.controller = controller

        self._name_label = tk.Label(self, text="Project", bg="#DADADA", fg="black")
        self._name_label.pack(anchor="w", padx=12, pady=10)

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """
        Update the page to reflect the currently selected project.

        :param project: Project configuration dictionary or 'None'.
        """
        if project is None:
            self._name_label.configure(text="Project")
            return

        name = project.get("robogym_project", {}).get("name")
        if not name:
            name = project.get("name", "Project")
        self._name_label.configure(text=str(name))
