import shutil
import tkinter as tk
from tkinter import font, messagebox

from mil_robogym.data_collection.get_all_project_config import (
    find_projects_dir,
    get_all_project_config,
)
from mil_robogym.data_collection.utils import to_lower_snake_case

from .project_row import ProjectRow


class StartPage(tk.Frame):
    """
    Start page of the MIL RoboGYM application.

    This page lists all available RoboGym projects and provides
    navigation to open an existing project or create a new one.
    """

    def __init__(self, parent, controller=None):
        """
        Initialize the start page UI.

        :param parent: Parent Tkinter widget.
        :param controller: Application controller used for page navigation.
        """

        super().__init__(parent, bg="#DADADA")
        self.controller = controller
        self._row_font = font.Font(family="Arial", size=15)

        # Fonts
        title_font = font.Font(family="Arial", size=20, weight="bold")
        button_font = font.Font(family="Arial", size=16)

        # Main layout container
        container = tk.Frame(self, bg="#DADADA")
        container.pack(fill="both", expand=True, padx=14, pady=14)

        container.grid_rowconfigure(0, weight=0)  # title
        container.grid_rowconfigure(1, weight=1)  # list area
        container.grid_rowconfigure(2, weight=0)  # bottom button
        container.grid_columnconfigure(0, weight=1)

        # Title
        title = tk.Label(
            container,
            text="MIL RoboGYM",
            bg="#DADADA",
            fg="black",
            font=title_font,
            anchor="w",
        )
        title.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        # List area
        self.list_area = tk.Frame(container, bg="#DADADA")
        self.list_area.grid(row=1, column=0, sticky="nsew")
        self.list_area.grid_columnconfigure(0, weight=1)
        self._render_projects()

        # Bottom button (Create Project +)
        bottom = tk.Frame(container, bg="#DADADA")
        bottom.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        bottom.grid_columnconfigure(0, weight=1)

        create_btn = tk.Button(
            bottom,
            text="Create Project +",
            font=button_font,
            relief="solid",
            bd=1,
            bg="#E6E6E6",
            activebackground="#DCDCDC",
            fg="black",
            padx=10,
            pady=6,
            command=self._on_create_project,
            cursor="hand2",
        )
        create_btn.grid(row=0, column=0, sticky="ew", pady=(0, 0))

    def _on_project(self, project):
        """
        Handle selection of an existing project. Navigates to the project view page and sets the active project context.

        :param project: Project configuration dictionary.
        """
        self.controller.show_page("view_project", project=project)

    def _on_create_project(self):
        """
        Navigate to the project creation page.
        """
        self.controller.show_page("create_project")

    def _on_delete_project(self, project: dict) -> None:
        """
        Delete a project folder after explicit confirmation.

        :param project: Project configuration dictionary.
        """
        name = project.get("robogym_project", {}).get("name", "").strip()
        if not name:
            return

        should_delete = messagebox.askyesno(
            title="Delete Project",
            message=(
                f"Delete project '{name}'?\n"
                "This will permanently remove all demos and agents in this project."
            ),
            icon="warning",
        )
        if not should_delete:
            return

        projects_dir = find_projects_dir()
        project_dir = projects_dir / to_lower_snake_case(name)
        if project_dir.exists():
            shutil.rmtree(project_dir)

        self._render_projects()

    def set_context(self, **_kwargs):
        """
        Refresh projects whenever this page is shown.
        """
        self._render_projects()

    def _render_projects(self):
        for child in self.list_area.winfo_children():
            child.destroy()

        projects = get_all_project_config()
        valid_projects = [
            project
            for project in projects
            if project.get("robogym_project", {}).get("name", "")
        ]

        if not valid_projects:
            empty = tk.Label(
                self.list_area,
                text="No projects found.",
                bg="#DADADA",
                fg="#444444",
                anchor="w",
            )
            empty.grid(row=0, column=0, sticky="w", pady=4)
            return

        for i, project in enumerate(valid_projects):
            name = project["robogym_project"]["name"]
            demos = project["num_demos"]

            row = ProjectRow(
                self.list_area,
                name,
                f"{demos} demonstrations",
                command=lambda p=project: self._on_project(p),
                action_text="Delete",
                action_command=lambda p=project: self._on_delete_project(p),
            )
            for child in row.winfo_children():
                for grandchild in child.winfo_children():
                    if isinstance(grandchild, (tk.Label, tk.Button)):
                        grandchild.configure(font=self._row_font)

            row.grid(row=i, column=0, sticky="ew", pady=4)
