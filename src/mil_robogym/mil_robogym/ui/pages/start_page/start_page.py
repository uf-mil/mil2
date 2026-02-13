import tkinter as tk
from tkinter import font

from mil_robogym.data_collection.get_all_project_config import get_all_project_config

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

        # Fonts
        title_font = font.Font(family="Arial", size=18, weight="bold")
        row_font = font.Font(family="Arial", size=11)
        button_font = font.Font(family="Arial", size=11)

        # Main layout container
        container = tk.Frame(self, bg="#DADADA")
        container.pack(fill="both", expand=True, padx=12, pady=10)

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
        title.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        # List area
        list_area = tk.Frame(container, bg="#DADADA")
        list_area.grid(row=1, column=0, sticky="nsew")
        list_area.grid_columnconfigure(0, weight=1)

        projects = get_all_project_config()

        if not projects:
            empty = tk.Label(
                list_area,
                text="No projects found.",
                bg="#DADADA",
                fg="#444444",
                anchor="w",
            )
            empty.grid(row=0, column=0, sticky="w", pady=4)
        else:
            for i, project in enumerate(projects):
                name = project["robogym_project"]["name"]
                demos = project["num_demos"]

                row = ProjectRow(
                    list_area,
                    name,
                    f"{demos} demonstrations",
                    command=lambda p=project: self._on_project(p),
                )
                for child in row.winfo_children():
                    for grandchild in child.winfo_children():
                        if isinstance(grandchild, tk.Label):
                            grandchild.configure(font=row_font)

                row.grid(row=i, column=0, sticky="ew", pady=4)

        # Bottom button (Create Project +)
        bottom = tk.Frame(container, bg="#DADADA")
        bottom.grid(row=2, column=0, sticky="ew", pady=(12, 0))
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
            padx=12,
            pady=10,
            command=self._on_create_project,
        )
        create_btn.grid(row=0, column=0, sticky="ew")

    def _on_project(self, project):
        """
        Handle selection of an existing project. Navigates to the project page and sets the active project context.

        :param project: Project configuration dictionary.
        """
        self.controller.show_page("project", project=project)

    def _on_create_project(self):
        """
        Navigate to the project creation page.
        """
        self.controller.show_page("create_project")
