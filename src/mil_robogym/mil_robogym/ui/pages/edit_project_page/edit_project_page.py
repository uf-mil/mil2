from __future__ import annotations

import tkinter as tk
from typing import Any, Mapping

from mil_robogym.clients.model_pose_client import ModelPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls import TeleopGUI
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame


class EditProjectPage(tk.Frame):
    """
    UI page for editing project configuration.

    This page intentionally mirrors the visual structure and interaction logic
    used by CreateProjectPage for random spawn, coordinate entry enable/disable,
    simulation coordinate grabbing, and topic selection lists.
    """

    def __init__(self, parent: tk.Widget, controller: Any | None = None) -> None:
        super().__init__(parent, bg="#DADADA")

        self.controller = controller

        self.coordinate1: tuple[float, float, float, float] | None = None
        self.coordinate2: tuple[float, float, float, float] | None = None

        self.world_control_client = WorldControlClient()
        self.gz_pose_client = ModelPoseClient()

        self.keyboard_controls_gui: TeleopGUI | None = None
        self.popup: GrabCoordinatesPopup | None = None

        self._world_default = self._safe_get_world_file()

        self._current_project_name = "Project"
        self.input_topics_selected: list[str] = []
        self.output_topics_selected: list[str] = []

        title_row = tk.Frame(self, bg="#DADADA")
        title_row.grid(row=0, column=0, columnspan=6, sticky="w", padx=14, pady=(14, 8))

        home_title = tk.Label(
            title_row,
            text="MIL RoboGYM >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        home_title.pack(side="left")
        home_title.configure(cursor="hand2")
        home_title.bind("<Button-1>", self._on_home_title_click)

        self.project_title = tk.Label(
            title_row,
            text=f"{self._current_project_name} > Edit Project",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        self.project_title.pack(side="left", padx=(6, 0))

        project_name_label = tk.Label(
            self,
            text="Project Name:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        )
        project_name_label.grid(row=1, column=0, sticky="w", padx=(14, 8), pady=5)

        self.project_name_var = tk.StringVar()
        project_name_entry = tk.Entry(
            self,
            textvariable=self.project_name_var,
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

        tk.Label(
            self,
            text="World File:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        ).grid(
            row=2,
            column=0,
            sticky="w",
            padx=(14, 8),
            pady=5,
        )
        self.world_file_var = tk.StringVar(value=self._world_default)
        tk.Entry(
            self,
            textvariable=self.world_file_var,
            font=("Arial", 15),
            state=tk.DISABLED,
        ).grid(
            row=2,
            column=1,
            columnspan=4,
            sticky="nsew",
            padx=(0, 8),
            pady=5,
            ipady=3,
        )

        tk.Label(
            self,
            text="Model Name:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        ).grid(
            row=3,
            column=0,
            sticky="w",
            padx=(14, 8),
            pady=5,
        )
        self.model_name_var = tk.StringVar(value="sub9")
        tk.Entry(
            self,
            textvariable=self.model_name_var,
            font=("Arial", 15),
            state=tk.DISABLED,
        ).grid(
            row=3,
            column=1,
            columnspan=3,
            sticky="nsew",
            padx=(0, 8),
            pady=5,
            ipady=3,
        )

        self.random_spawn_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            self,
            text="Random Spawn Space:",
            variable=self.random_spawn_var,
            command=self._toggle_random_spawn,
            bg="#DADADA",
            fg="black",
            activebackground="#DADADA",
            font=("Arial", 15, "bold"),
            highlightthickness=0,
            anchor="w",
            borderwidth=0,
        ).grid(row=4, column=0, columnspan=2, sticky="nsew", padx=(14, 8), pady=(6, 2))

        self.coord1_label = tk.Label(
            self,
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
        self.coord1_entry = tk.Entry(
            self,
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
            self,
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
        self.coord2_entry = tk.Entry(
            self,
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
            self,
            text="Grab from Sim",
            command=self._on_grab_from_sim,
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

        tk.Label(
            self,
            text="Input Topics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        ).grid(
            row=6,
            column=0,
            columnspan=2,
            sticky="w",
            padx=14,
            pady=(6, 2),
        )
        tk.Label(
            self,
            text="Output Topics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        ).grid(
            row=6,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(8, 14),
            pady=(6, 2),
        )

        outer = tk.Frame(self, bg="#DADADA")
        outer.grid(row=7, column=0, columnspan=6, sticky="nsew", padx=14, pady=(0, 8))
        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)
        outer.grid_rowconfigure(0, weight=1)

        self.input_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        self.input_topic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        self.output_topic_frame.grid(row=0, column=1, sticky="nsew")
        self._render_topics(list_type="input", topics=[])
        self._render_topics(list_type="output", topics=[])

        tk.Button(
            self,
            text="Cancel",
            command=self._on_cancel,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 16),
            padx=10,
            pady=6,
            cursor="hand2",
        ).grid(row=8, column=0, columnspan=3, sticky="nsew", padx=(14, 8), pady=(8, 14))

        tk.Button(
            self,
            text="Save Changes",
            command=self._on_save_changes,
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
            row=8,
            column=3,
            columnspan=3,
            sticky="nsew",
            padx=(8, 14),
            pady=(8, 14),
        )

        for col in range(6):
            self.grid_columnconfigure(col, weight=1, uniform="half")
        self.grid_rowconfigure(7, weight=1)

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """Populate form fields from a project payload."""
        if project is None:
            self._set_project_title("Project")
            self.project_name_var.set("")
            self.world_file_var.set(self._world_default)
            self.model_name_var.set("sub9")
            self.random_spawn_var.set(False)
            self.coord1_var.set("")
            self.coord2_var.set("")
            self.coordinate1 = None
            self.coordinate2 = None
            self.input_topics_selected = []
            self.output_topics_selected = []
            self._render_topics(list_type="input", topics=self.input_topics_selected)
            self._render_topics(list_type="output", topics=self.output_topics_selected)
            self._toggle_random_spawn()
            return

        details = project.get("robogym_project", {})
        print("details", details)

        project_name = str(details.get("name", "Project"))
        self._set_project_title(project_name)
        self.project_name_var.set(project_name)

        self.world_file_var.set(str(details.get("world_file", self._world_default)))
        self.model_name_var.set(str(details.get("model_name", "sub9")))

        random_spawn = details.get("random_spawn_space", {})
        enabled = bool(random_spawn.get("enabled", False))
        coord1 = self._normalize_coord(random_spawn.get("coord1_4d"))
        coord2 = self._normalize_coord(random_spawn.get("coord2_4d"))

        self.coordinate1 = coord1
        self.coordinate2 = coord2

        self.coord1_var.set(self._format_coord(coord1))
        self.coord2_var.set(self._format_coord(coord2))

        self.random_spawn_var.set(enabled)
        self._toggle_random_spawn()

        input_topics = details.get("input_topics", [])
        output_topics = details.get("output_topics", [])
        self.input_topics_selected = [str(topic) for topic in (input_topics or [])]
        self.output_topics_selected = [str(topic) for topic in (output_topics or [])]
        self._render_topics(list_type="input", topics=self.input_topics_selected)
        self._render_topics(list_type="output", topics=self.output_topics_selected)

    def _safe_get_project_by_name(self, name: str) -> Mapping[str, Any] | None:
        if not name:
            return None

        projects = get_all_project_config()
        for project in projects:
            details = project.get("robogym_project", {})
            if details.get("name") == name:
                return project
        return None

    def _set_project_title(self, project_name: str) -> None:
        self._current_project_name = project_name if project_name else "Project"
        self.project_title.configure(
            text=f"{self._current_project_name} > Edit Project",
        )

    def _safe_get_world_file(self) -> str:
        return "~/mil2/install/subjugator_gazebo/share/subjugator_gazebo/worlds/robosub_2025.world"

    def _toggle_random_spawn(self) -> None:
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

    def _render_topics(self, list_type: str, topics: list[str]) -> None:
        if list_type == "input":
            frame = self.input_topic_frame.content
        else:
            frame = self.output_topic_frame.content

        for child in frame.winfo_children():
            child.destroy()

        # if not topics:
        #     raise ValueError("Topics list for project when editing it is empty.")

        for topic in topics:
            tk.Label(
                frame,
                text=f"• {topic}",
                bg="#DADADA",
                fg="black",
                font=("Arial", 14),
                anchor="w",
            ).pack(anchor="w")

    def _on_grab_from_sim(self) -> None:
        self.world_control_client.play_simulation()

        self.keyboard_controls_gui = self.keyboard_controls_gui or TeleopGUI(
            self,
            self._on_close_of_keyboard_controls,
        )
        self.keyboard_controls_gui.show()

        if self.popup and self.popup.win.winfo_exists():
            self.popup.win.lift()
            self.popup.win.focus_force()
            self.keyboard_controls_gui.show()
            return

        self.popup = GrabCoordinatesPopup(
            self,
            self.gz_pose_client.send_request,
            self._display_collected_coords,
        )

    def _display_collected_coords(
        self,
        c1: tuple[float, float, float, float] | None,
        c2: tuple[float, float, float, float] | None,
    ) -> None:
        if c1 and c2:
            self.coordinate1 = c1
            self.coord1_entry.delete(0, tk.END)
            self.coord1_entry.insert(0, f"{tuple(round(v, 1) for v in c1)}")

            self.coordinate2 = c2
            self.coord2_entry.delete(0, tk.END)
            self.coord2_entry.insert(0, f"{tuple(round(v, 1) for v in c2)}")

        if self.keyboard_controls_gui is not None:
            self.keyboard_controls_gui.hide()
        self.world_control_client.pause_simulation()
        self.popup = None

    def _on_close_of_keyboard_controls(self) -> None:
        if self.popup is not None:
            self.popup.finish()

    def _on_cancel(self) -> None:
        print("activation for button cancel")

    def _on_save_changes(self) -> None:
        print("activation for button save changes")

    def _on_home_title_click(self, _event: tk.Event | None = None) -> None:
        if self.controller is not None:
            self.controller.show_page("start")

    def _format_coord(self, coord: tuple[float, float, float, float] | None) -> str:
        if coord is None:
            return ""
        return f"{tuple(round(v, 1) for v in coord)}"

    def _normalize_coord(self, value: Any) -> tuple[float, float, float, float] | None:
        if value is None:
            return None

        if isinstance(value, (tuple, list)) and len(value) == 4:
            try:
                return tuple(float(v) for v in value)
            except (TypeError, ValueError):
                return None

        if isinstance(value, str):
            return self._parse_coord(value)

        return None

    def _parse_coord(self, value: str) -> tuple[float, float, float, float] | None:
        cleaned = value.strip().strip("()")
        parts = [p.strip() for p in cleaned.split(",")]
        if len(parts) != 4:
            return None
        try:
            return tuple(float(p) for p in parts)
        except ValueError:
            return None
