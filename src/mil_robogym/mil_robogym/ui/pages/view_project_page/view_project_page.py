from __future__ import annotations

import csv
import tkinter as tk
from tkinter import messagebox
from typing import Any, Mapping

from mil_robogym.data_collection.delete_demo import delete_demo
from mil_robogym.data_collection.filesystem import get_demo_dir_path
from mil_robogym.data_collection.get_all_demo_config import get_all_demo_config
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.ui.components.create_demo_popup import CreateDemoPopup
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame


class ViewProjectPage(tk.Frame):
    """
    UI page for viewing a project's demos and top-level actions.

    This page mirrors the visual structure used by CreateProjectPage and is
    populated using project data from 'get_all_project_config'.
    """

    def __init__(self, parent: tk.Widget, controller: Any | None = None) -> None:
        """
        Initialize the project detail page.

        :param parent: Parent Tkinter widget that owns this frame.
        :param controller: Optional page controller that supports 'show_page'.
        """
        super().__init__(parent, bg="#DADADA")
        self.controller = controller

        self.create_demo_popup = None

        self.project = None
        self.project_name = "Project"
        self._num_demos = 0
        self._demos = []
        self._demo_names: list[str] = []

        self._title_row = tk.Frame(self, bg="#DADADA")
        self._title_row.grid(
            row=0,
            column=0,
            columnspan=6,
            sticky="nsew",
            padx=14,
            pady=(14, 8),
        )
        self._title_row.grid_columnconfigure(0, weight=1)
        self._title_row.grid_columnconfigure(1, weight=0)

        self._title_left = tk.Frame(self._title_row, bg="#DADADA")
        self._title_left.grid(row=0, column=0, sticky="w")

        self._home_title = tk.Label(
            self._title_left,
            text="MIL RoboGYM >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        self._home_title.pack(side="left")
        self._home_title.configure(cursor="hand2")
        self._home_title.bind("<Button-1>", self._on_home_title_click)

        self._page_title = tk.Label(
            self._title_left,
            text=self.project_name,
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        self._page_title.pack(side="left", padx=(6, 0))

        tk.Button(
            self._title_row,
            text="Train/Test",
            command=self._on_train_test,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 15),
            padx=16,
            pady=4,
            cursor="hand2",
        ).grid(row=0, column=1, sticky="e")

        self._list_area = ScrollableFrame(self, bg="#DADADA", fill_height=True)
        self._list_area.grid(
            row=1,
            column=0,
            columnspan=6,
            sticky="nsew",
            padx=14,
            pady=(0, 8),
        )
        self._list_area.content.grid_columnconfigure(0, weight=1)

        self._render_demo_rows()

        tk.Button(
            self,
            text="Edit Project",
            command=self._on_editproject,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 16),
            padx=10,
            pady=6,
            cursor="hand2",
        ).grid(
            row=2,
            column=0,
            columnspan=3,
            sticky="nsew",
            padx=(14, 8),
            pady=(8, 14),
        )

        tk.Button(
            self,
            text="Record Demo +",
            command=self._on_record_demo,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 16),
            padx=10,
            pady=6,
            cursor="hand2",
        ).grid(
            row=2,
            column=3,
            columnspan=3,
            sticky="nsew",
            padx=(8, 14),
            pady=(8, 14),
        )

        for col in range(6):
            self.grid_columnconfigure(col, weight=1, uniform="half")
        self.grid_rowconfigure(1, weight=1)

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """
        Update this page for the selected project.

        Loads project information from 'get_all_project_config' and updates
        the title and demo rows.

        :param project: Selected project dictionary, usually passed from StartPage.
        """
        self.project = project

        if project is None:
            self.project_name = "Project"
            self._num_demos = 0
            self._demo_names = []
            self._page_title.configure(text=self.project_name)
            self._render_demo_rows()
            return

        selected_name = project.get("robogym_project", {}).get("name", "")
        loaded = self._safe_get_project_by_name(str(selected_name))

        if loaded is None:
            self.project_name = str(selected_name) if selected_name else "Project"
            self._num_demos = int(project.get("num_demos", 0))
        else:
            self.project = loaded
            self.project_name = loaded.get("robogym_project", {}).get(
                "name",
                "Project",
            )
            self._num_demos = int(loaded.get("num_demos", 0))

        self._demo_names = self._safe_get_demo_names(self.project_name)
        self._num_demos = len(self._demo_names)

        self._page_title.configure(text=self.project_name)
        self._render_demo_rows()
        self._reset_demo_list_scroll()

    def _safe_get_project_by_name(self, name: str) -> dict[str, Any] | None:
        """
        Find a project by name from all available project configs.

        :param name: Display name of the project.
        :return: Matching project config if found, else 'None'.
        """
        if not name:
            return None

        projects = get_all_project_config()
        for project in projects:
            if project.get("robogym_project", {}).get("name") == name:
                return project
        return None

    def _safe_get_demo_names(self, project_name: str) -> list[str]:
        """
        Load demo names for a project.

        :param project_name: Display name of the project.
        :return: Sorted list of raw demo names from demo config files.
        """
        if not project_name:
            return []

        try:
            self._demos = dict(sorted(get_all_demo_config(project_name).items()))
        except (FileNotFoundError, ValueError) as e:
            raise RuntimeError(
                f"Get all demo config for '{project_name}' failed.",
            ) from e

        return self._demos.keys()

    def _render_demo_rows(self) -> None:
        """
        Render the list of demo rows for the active project.
        """
        for child in self._list_area.content.winfo_children():
            child.destroy()

        if self._num_demos <= 0:
            tk.Label(
                self._list_area.content,
                text="No demos found.",
                bg="#DADADA",
                fg="#444444",
                font=("Arial", 14),
                anchor="w",
            ).grid(row=0, column=0, sticky="w", pady=4)
            return

        for index, (demo_name, demo) in enumerate(self._demos.items()):
            step_count = self._get_demo_step_count(demo)
            row = tk.Frame(
                self._list_area.content,
                bg="#ECECEC",
                bd=1,
                relief="solid",
                highlightthickness=0,
                cursor="hand2",
            )
            row.grid(row=index, column=0, sticky="ew", pady=4)
            row.grid_columnconfigure(0, weight=1)
            row.grid_columnconfigure(1, weight=0)
            row.grid_columnconfigure(2, weight=0)

            left_label = tk.Label(
                row,
                text=demo_name,
                bg="#ECECEC",
                fg="black",
                font=("Arial", 15),
                anchor="w",
                padx=8,
                pady=6,
                cursor="hand2",
            )
            left_label.grid(row=0, column=0, sticky="w")

            right_label = tk.Label(
                row,
                text=f"{step_count} steps",
                bg="#ECECEC",
                fg="black",
                font=("Arial", 15),
                anchor="e",
                padx=8,
                pady=6,
                cursor="hand2",
            )
            right_label.grid(row=0, column=1, sticky="e")

            delete_button = tk.Button(
                row,
                text="Delete",
                command=lambda n=demo_name, d=demo: self._on_delete_demo(n, d),
                bg="#ECECEC",
                activebackground="#DFDFDF",
                fg="black",
                relief="solid",
                bd=1,
                font=("Arial", 12),
                padx=8,
                pady=4,
                cursor="hand2",
            )
            delete_button.grid(row=0, column=2, sticky="e", padx=(0, 8))

            def on_click(
                _event: tk.Event | None = None,
                name: str = demo_name,
                demo: dict = demo,
            ) -> None:
                self._on_demo_row_click(name, demo)

            def on_enter(
                _event: tk.Event | None = None,
                widgets: tuple[tk.Widget, tk.Widget, tk.Widget] = (
                    row,
                    left_label,
                    right_label,
                ),
            ) -> None:
                for widget in widgets:
                    widget.configure(bg="#DCDCDC")

            def on_leave(
                _event: tk.Event | None = None,
                widgets: tuple[tk.Widget, tk.Widget, tk.Widget] = (
                    row,
                    left_label,
                    right_label,
                ),
            ) -> None:
                for widget in widgets:
                    widget.configure(bg="#ECECEC")

            for widget in (row, left_label, right_label):
                widget.bind("<Button-1>", on_click)
                widget.bind("<Enter>", on_enter)
                widget.bind("<Leave>", on_leave)

    def _reset_demo_list_scroll(self) -> None:
        if hasattr(self, "_list_area"):
            self._list_area.reset_scroll()

    def _get_demo_step_count(self, demo: Mapping[str, Any]) -> int:
        """Return number of steps as (actions CSV rows - header row)."""
        if not self.project_name:
            return 0

        try:
            demo_cfg = demo.get("robogym_demo", {})
            demo_dir = get_demo_dir_path({"name": self.project_name}, demo_cfg)
            actions_csv = demo_dir / "data" / "actions" / "data.csv"
            if not actions_csv.is_file():
                return 0

            with actions_csv.open("r", encoding="utf-8", newline="") as csv_file:
                row_count = sum(1 for _ in csv.reader(csv_file))

            return max(row_count - 1, 0)
        except (KeyError, OSError, TypeError, ValueError, csv.Error):
            return 0

    def _on_home_title_click(self, _event: tk.Event | None = None) -> None:
        """Navigate back to the start page."""
        if self.controller is not None:
            self.controller.show_page("start")

    def _on_train_test(self) -> None:
        """Handle Train/Test button action."""
        if self.controller is None:
            return

        project_payload = self._safe_get_project_by_name(self.project_name)
        if project_payload is None:
            raise FileNotFoundError(
                f"Project not found under name {self.project_name}",
            )

        self.controller.show_page("train_test", project=project_payload)

    def _on_editproject(self) -> None:
        """Handle Edit Project button action."""
        if self.controller is None:
            return

        project_payload = self._safe_get_project_by_name(self.project_name)
        if project_payload is None:
            raise FileNotFoundError(
                f"Project not found under name {self.project_name}",
            )

        self.controller.show_page("edit_project", project=project_payload)

    def _on_record_demo(self) -> None:
        """Handle Record Demo button action."""

        if self.create_demo_popup and self.create_demo_popup.win.winfo_exists():
            self.create_demo_popup.win.lift()
            self.create_demo_popup.win.focus_force()
            return

        def remove_reference_to_popup():
            self.create_demo_popup = None

        def handle_demo_created(demo_name: str, demo_cfg: dict[str, Any]) -> None:
            self.create_demo_popup = None
            self.controller.show_page(
                "view_demo",
                project=self.project,
                demo_name=demo_name,
                demo=demo_cfg,
            )

        self.create_demo_popup = CreateDemoPopup(
            self,
            project=self.project["robogym_project"],
            on_created=handle_demo_created,
            on_cancel=remove_reference_to_popup,
        )

    def _on_demo_row_click(self, demo_name, demo) -> None:
        """
        Handle clicking a demo row.

        :param demo_name: Human-readable demo name, for example 'Demo 1'.
        """
        self.controller.show_page(
            "view_demo",
            project=self.project,
            demo_name=demo_name,
            demo=demo,
        )

    def _on_delete_demo(self, demo_name: str, demo: Mapping[str, Any]) -> None:
        """
        Delete a demo folder after explicit confirmation.

        :param demo_name: Human-readable demo name.
        :param demo: Demo config dictionary.
        """
        if not self.project_name or not isinstance(self.project, Mapping):
            return

        should_delete = messagebox.askyesno(
            title="Delete Demo",
            message=f"Delete demo '{demo_name}'?\nThis action cannot be undone.",
            icon="warning",
        )
        if not should_delete:
            return

        demo_cfg = demo.get("robogym_demo", {}) if isinstance(demo, Mapping) else {}
        if "name" not in demo_cfg:
            demo_cfg = {**demo_cfg, "name": demo_name}

        delete_demo(self.project, {"robogym_demo": demo_cfg})

        self._demo_names = list(self._safe_get_demo_names(self.project_name))
        self._num_demos = len(self._demo_names)
        self._render_demo_rows()
        self._reset_demo_list_scroll()
