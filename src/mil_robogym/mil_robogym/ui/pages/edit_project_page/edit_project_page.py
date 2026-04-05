from __future__ import annotations

import tkinter as tk
from tkinter import messagebox
from typing import Any, Mapping

from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.build_tensor_spec import build_tensor_spec
from mil_robogym.data_collection.filesystem import edit_project
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.data_collection.ros_graph import get_topic_names
from mil_robogym.data_collection.types import (
    Coord4D,
    NonNumericTopicFieldSelection,
    RoboGymProjectYaml,
)
from mil_robogym.data_collection.utils import (
    filter_populated_non_numeric_topic_fields,
)
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI
from mil_robogym.ui.pages.create_project_page.sub_topics_section import (
    SubTopicsSection,
)
from mil_robogym.ui.pages.create_project_page.topics_section import TopicsSection


class EditProjectPage(tk.Frame):
    """UI page for editing project configuration."""

    def __init__(self, parent: tk.Widget, controller: Any | None = None) -> None:
        super().__init__(parent, bg="#DADADA")

        self.controller = controller

        self.coordinate1: tuple[float, float, float, float] | None = None
        self.coordinate2: tuple[float, float, float, float] | None = None

        self.world_control_client = WorldControlClient()
        self.gz_pose_client = GetPoseClient()

        self.keyboard_controls_gui: KeyboardControlsGUI | None = None
        self.popup: GrabCoordinatesPopup | None = None

        self._world_default = self._safe_get_world_file()
        self._available_topics = self._safe_get_topics()

        self._current_project_name = "Project"
        self._original_project_name = "Project"
        self._current_tensor_spec: Mapping[str, Any] | None = None
        self._topic_edit_locked = False

        self.input_topics_selected: list[str] = []
        self.output_topics_selected: list[str] = []
        self.input_topic_subtopics: dict[str, list[str]] = {}
        self.output_topic_subtopics: dict[str, list[str]] = {}
        self.input_non_numeric_topics: dict[
            str,
            list[NonNumericTopicFieldSelection],
        ] = {}
        self.output_non_numeric_topics: dict[
            str,
            list[NonNumericTopicFieldSelection],
        ] = {}
        self._original_input_topic_subtopics: dict[str, list[str]] = {}
        self._original_output_topic_subtopics: dict[str, list[str]] = {}
        self._original_input_non_numeric_topics: dict[
            str,
            list[NonNumericTopicFieldSelection],
        ] = {}
        self._original_output_non_numeric_topics: dict[
            str,
            list[NonNumericTopicFieldSelection],
        ] = {}

        self._build_layout()

    def _build_layout(self) -> None:
        title_row = tk.Frame(self, bg="#DADADA")
        title_row.grid(
            row=0,
            column=0,
            columnspan=6,
            sticky="w",
            padx=14,
            pady=(14, 8),
        )

        home_title = tk.Label(
            title_row,
            text="MIL RoboGYM >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
            cursor="hand2",
        )
        home_title.pack(side="left")
        home_title.bind("<Button-1>", self._on_home_title_click)

        self.project_title = tk.Label(
            title_row,
            text=self._current_project_name,
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
            cursor="hand2",
        )
        self.project_title.pack(side="left", padx=(6, 0))
        self.project_title.bind("<Button-1>", self._on_project_title_click)

        tk.Label(
            title_row,
            text="> Edit Project",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        ).pack(side="left", padx=(6, 0))

        tk.Label(
            self,
            text="Project Name:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        ).grid(row=1, column=0, sticky="w", padx=(14, 8), pady=5)

        self.project_name_var = tk.StringVar()
        tk.Entry(
            self,
            textvariable=self.project_name_var,
            font=("Arial", 15),
        ).grid(
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
        ).grid(row=2, column=0, sticky="w", padx=(14, 8), pady=5)
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
        ).grid(row=3, column=0, sticky="w", padx=(14, 8), pady=5)
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

        self.topics_section = TopicsSection(
            self,
            self._available_topics,
            self._on_topics_selection_changed,
            self._on_refresh_topics,
        )
        self.sub_topics_section = SubTopicsSection(
            self,
            self._on_subtopics_selection_changed,
        )

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
        ).grid(
            row=10,
            column=0,
            columnspan=3,
            sticky="nsew",
            padx=(14, 8),
            pady=(8, 14),
        )

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
            row=10,
            column=3,
            columnspan=3,
            sticky="nsew",
            padx=(8, 14),
            pady=(8, 14),
        )

        for col in range(6):
            self.grid_columnconfigure(col, weight=1, uniform="half")
        self.grid_rowconfigure(7, weight=1)
        self.grid_rowconfigure(9, weight=1)

    def _apply_topic_edit_lock_state(self) -> None:
        topics_enabled = not self._topic_edit_locked
        self.topics_section.set_selection_enabled(topics_enabled)
        self.sub_topics_section.set_selection_enabled(topics_enabled)

        if self._topic_edit_locked:
            label_suffix = " (locked: existing demos)"
            label_color = "#666666"
        else:
            label_suffix = ""
            label_color = "black"

        self.topics_section.input_topics_label.configure(
            text=f"Input Topics{label_suffix}",
            fg=label_color,
        )
        self.topics_section.output_topics_label.configure(
            text=f"Output Topics{label_suffix}",
            fg=label_color,
        )
        self.sub_topics_section.input_subtopics_label.configure(
            text=f"Input Subtopics{label_suffix}",
            fg=label_color,
        )
        self.sub_topics_section.output_subtopics_label.configure(
            text=f"Output Subtopics{label_suffix}",
            fg=label_color,
        )

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """Populate form fields from a project payload."""
        self._available_topics = self._safe_get_topics()

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
            self._original_project_name = "Project"
            self._current_tensor_spec = None
            self._topic_edit_locked = False
            self.input_topics_selected = []
            self.output_topics_selected = []
            self.input_topic_subtopics = {}
            self.output_topic_subtopics = {}
            self.input_non_numeric_topics = {}
            self.output_non_numeric_topics = {}
            self._snapshot_original_topic_selection()
            self.topics_section.set_topics(self._available_topics)
            self.topics_section.set_selected_topics(input_topics=[], output_topics=[])
            self.sub_topics_section.set_selected_topics(
                input_topics=[],
                output_topics=[],
            )
            self.sub_topics_section.set_selected_fields(
                input_numeric_fields={},
                output_numeric_fields={},
                input_non_numeric_fields={},
                output_non_numeric_fields={},
            )
            self._apply_topic_edit_lock_state()
            self._toggle_random_spawn()
            return

        details = project.get("robogym_project", {})

        project_name = str(details.get("name", "Project"))
        self._set_project_title(project_name)
        self._original_project_name = project_name
        self.project_name_var.set(project_name)

        self.world_file_var.set(str(details.get("world_file", self._world_default)))
        self.model_name_var.set(str(details.get("model_name", "sub9")))
        tensor_spec = details.get("tensor_spec")
        self._current_tensor_spec = (
            tensor_spec if isinstance(tensor_spec, Mapping) else None
        )

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

        self.input_topic_subtopics = self._normalize_topic_subtopics(
            details.get("input_topics", {}),
        )
        self.output_topic_subtopics = self._normalize_topic_subtopics(
            details.get("output_topics", {}),
        )
        self.input_non_numeric_topics = self._normalize_non_numeric_topics(
            details.get("input_non_numeric_topics", {}),
        )
        self.output_non_numeric_topics = self._normalize_non_numeric_topics(
            details.get("output_non_numeric_topics", {}),
        )

        self.input_topics_selected = self._ordered_union(
            list(self.input_topic_subtopics),
            list(self.input_non_numeric_topics),
        )
        self.output_topics_selected = self._ordered_union(
            list(self.output_topic_subtopics),
            list(self.output_non_numeric_topics),
        )
        self._snapshot_original_topic_selection()
        self._topic_edit_locked = int(project.get("num_demos", 0)) > 0

        if self._topic_edit_locked:
            editable_topics = self._ordered_union(
                self.input_topics_selected,
                self.output_topics_selected,
            )
        else:
            editable_topics = self._ordered_union(
                self.input_topics_selected,
                self.output_topics_selected,
                self._available_topics,
            )
        self.topics_section.set_topics(editable_topics)
        self.topics_section.set_selected_topics(
            input_topics=self.input_topics_selected,
            output_topics=self.output_topics_selected,
        )
        self.sub_topics_section.set_selected_topics(
            input_topics=self.input_topics_selected,
            output_topics=self.output_topics_selected,
        )
        self.sub_topics_section.set_selected_fields(
            input_numeric_fields=self.input_topic_subtopics,
            output_numeric_fields=self.output_topic_subtopics,
            input_non_numeric_fields=self.input_non_numeric_topics,
            output_non_numeric_fields=self.output_non_numeric_topics,
        )
        self._apply_topic_edit_lock_state()

    def _safe_get_topics(self) -> list[str]:
        try:
            return get_topic_names()
        except RuntimeError as exc:
            raise RuntimeError(
                "Getting ROS 2 topics on edit project page failed",
            ) from exc

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
        self.project_title.configure(text=self._current_project_name)

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

    def _on_grab_from_sim(self) -> None:
        self.world_control_client.play_simulation()

        self.keyboard_controls_gui = self.keyboard_controls_gui or KeyboardControlsGUI(
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

    def _display_collected_coords(self, coords: list[Coord4D]) -> None:
        c1, c2 = coords

        if c1 and c2:
            self.coordinate1 = c1
            self.coord1_var.set(f"{tuple(round(v, 1) for v in c1)}")

            self.coordinate2 = c2
            self.coord2_var.set(f"{tuple(round(v, 1) for v in c2)}")

        if self.keyboard_controls_gui is not None:
            self.keyboard_controls_gui.hide()
        self.world_control_client.pause_simulation()
        self.popup = None

    def _on_close_of_keyboard_controls(self) -> None:
        if self.popup is not None:
            self.popup.finish()

    def _on_topics_selection_changed(self) -> None:
        if self._topic_edit_locked:
            return
        self.input_topics_selected = self.topics_section.get_selected_input_topics()
        self.output_topics_selected = self.topics_section.get_selected_output_topics()
        self.sub_topics_section.set_selected_topics(
            input_topics=self.input_topics_selected,
            output_topics=self.output_topics_selected,
        )
        self._sync_selected_topic_fields_from_section()

    def _on_subtopics_selection_changed(self) -> None:
        if self._topic_edit_locked:
            return
        self._sync_selected_topic_fields_from_section()

    def _on_refresh_topics(self) -> None:
        if self._topic_edit_locked:
            return
        self._available_topics = self._safe_get_topics()
        self.topics_section.set_topics(self._available_topics)
        self.input_topics_selected = self.topics_section.get_selected_input_topics()
        self.output_topics_selected = self.topics_section.get_selected_output_topics()
        self.sub_topics_section.set_selected_topics(
            input_topics=self.input_topics_selected,
            output_topics=self.output_topics_selected,
        )
        self.sub_topics_section.reload_selected_topics()
        self._sync_selected_topic_fields_from_section()

    def _sync_selected_topic_fields_from_section(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> None:
        self.input_topic_subtopics = (
            self.sub_topics_section.get_selected_input_topic_subtopics(
                ensure_loaded=ensure_loaded,
            )
        )
        self.output_topic_subtopics = (
            self.sub_topics_section.get_selected_output_topic_subtopics(
                ensure_loaded=ensure_loaded,
            )
        )
        self.input_non_numeric_topics = (
            self.sub_topics_section.get_selected_input_non_numeric_topic_fields(
                ensure_loaded=ensure_loaded,
            )
        )
        self.output_non_numeric_topics = (
            self.sub_topics_section.get_selected_output_non_numeric_topic_fields(
                ensure_loaded=ensure_loaded,
            )
        )

    def _on_cancel(self) -> None:
        should_exit = messagebox.askyesno(
            title="Cancel",
            message=(
                "Are you sure you want to exit Edit Project? Your changes "
                "will not be saved."
            ),
        )
        if should_exit and self.controller is not None:
            self.controller.show_page(
                "view_project",
                project={"robogym_project": {"name": self._current_project_name}},
            )

    def _on_save_changes(self) -> None:
        should_save = messagebox.askyesno(
            title="Save Changes",
            message="Do you want to save changes to this project?",
        )
        if not should_save:
            return

        project_cfg = self._build_project_config_from_form()
        if project_cfg is None:
            return

        try:
            edit_project(
                project_cfg,
                original_project_name=self._original_project_name,
            )
        except (FileNotFoundError, FileExistsError, RuntimeError, ValueError) as exc:
            messagebox.showerror("Save Changes", f"Failed to save project:\n{exc}")
            return

        saved_name = project_cfg["name"]
        loaded_project = self._safe_get_project_by_name(saved_name)
        if loaded_project is None:
            loaded_project = {"robogym_project": {"name": saved_name}}

        if self.controller is not None:
            self.controller.show_page("view_project", project=loaded_project)

    def _build_project_config_from_form(self) -> RoboGymProjectYaml | None:
        project_name = self.project_name_var.get().strip()
        if not project_name:
            messagebox.showerror("Save Changes", "Project name cannot be empty.")
            return None

        if self._topic_edit_locked:
            input_topics_selected = self._ordered_union(
                list(self._original_input_topic_subtopics),
                list(self._original_input_non_numeric_topics),
            )
            output_topics_selected = self._ordered_union(
                list(self._original_output_topic_subtopics),
                list(self._original_output_non_numeric_topics),
            )
            input_topic_subtopics = {
                topic: list(fields)
                for topic, fields in self._original_input_topic_subtopics.items()
            }
            output_topic_subtopics = {
                topic: list(fields)
                for topic, fields in self._original_output_topic_subtopics.items()
            }
            input_non_numeric_topics = {
                topic: [dict(field) for field in fields]
                for topic, fields in self._original_input_non_numeric_topics.items()
            }
            output_non_numeric_topics = {
                topic: [dict(field) for field in fields]
                for topic, fields in self._original_output_non_numeric_topics.items()
            }
        else:
            self.input_topics_selected = self.topics_section.get_selected_input_topics()
            self.output_topics_selected = (
                self.topics_section.get_selected_output_topics()
            )
            self._sync_selected_topic_fields_from_section(ensure_loaded=True)
            input_topics_selected = list(self.input_topics_selected)
            output_topics_selected = list(self.output_topics_selected)
            input_topic_subtopics = {
                topic: list(fields)
                for topic, fields in self.input_topic_subtopics.items()
            }
            output_topic_subtopics = {
                topic: list(fields)
                for topic, fields in self.output_topic_subtopics.items()
            }
            input_non_numeric_topics = {
                topic: [dict(field) for field in fields]
                for topic, fields in self.input_non_numeric_topics.items()
            }
            output_non_numeric_topics = {
                topic: [dict(field) for field in fields]
                for topic, fields in self.output_non_numeric_topics.items()
            }

        random_enabled = bool(self.random_spawn_var.get())
        if random_enabled:
            coord1 = self._normalize_coord(self.coord1_var.get())
            coord2 = self._normalize_coord(self.coord2_var.get())
            if coord1 is None or coord2 is None:
                messagebox.showerror(
                    "Save Changes",
                    "Random spawn is enabled, but one or both coordinates are invalid.",
                )
                return None
        else:
            coord1 = self.coordinate1 or (0.0, 0.0, 0.0, 0.0)
            coord2 = self.coordinate2 or (0.0, 0.0, 0.0, 0.0)

        project_cfg: RoboGymProjectYaml = {
            "name": project_name,
            "world_file": self.world_file_var.get().strip(),
            "model_name": self.model_name_var.get().strip(),
            "random_spawn_space": {
                "enabled": random_enabled,
                "coord1_4d": [float(v) for v in coord1],
                "coord2_4d": [float(v) for v in coord2],
            },
            "input_topics": {
                topic: list(input_topic_subtopics.get(topic, []))
                for topic in input_topics_selected
            },
            "output_topics": {
                topic: list(output_topic_subtopics.get(topic, []))
                for topic in output_topics_selected
            },
        }

        input_non_numeric_topics = filter_populated_non_numeric_topic_fields(
            {
                topic: fields
                for topic, fields in input_non_numeric_topics.items()
                if topic in input_topics_selected
            },
        )
        output_non_numeric_topics = filter_populated_non_numeric_topic_fields(
            {
                topic: fields
                for topic, fields in output_non_numeric_topics.items()
                if topic in output_topics_selected
            },
        )

        if input_non_numeric_topics:
            project_cfg["input_non_numeric_topics"] = input_non_numeric_topics
        if output_non_numeric_topics:
            project_cfg["output_non_numeric_topics"] = output_non_numeric_topics

        if self._topic_selection_changed():
            if self._has_any_numeric_topic_selection():
                try:
                    project_cfg["tensor_spec"] = build_tensor_spec(project_cfg)
                except (RuntimeError, ValueError, KeyError) as exc:
                    messagebox.showerror(
                        "Save Changes",
                        f"Failed to recompute tensor spec:\n{exc}",
                    )
                    return None
        elif self._current_tensor_spec is not None:
            project_cfg["tensor_spec"] = dict(self._current_tensor_spec)

        return project_cfg

    def _topic_selection_changed(self) -> bool:
        if self._topic_edit_locked:
            return False
        return (
            self.input_topic_subtopics != self._original_input_topic_subtopics
            or self.output_topic_subtopics != self._original_output_topic_subtopics
            or self._normalize_non_numeric_topic_map(self.input_non_numeric_topics)
            != self._normalize_non_numeric_topic_map(
                self._original_input_non_numeric_topics,
            )
            or self._normalize_non_numeric_topic_map(self.output_non_numeric_topics)
            != self._normalize_non_numeric_topic_map(
                self._original_output_non_numeric_topics,
            )
        )

    def _has_any_numeric_topic_selection(self) -> bool:
        return any(self.input_topic_subtopics.values()) and any(
            self.output_topic_subtopics.values(),
        )

    def _snapshot_original_topic_selection(self) -> None:
        self._original_input_topic_subtopics = {
            topic: list(fields) for topic, fields in self.input_topic_subtopics.items()
        }
        self._original_output_topic_subtopics = {
            topic: list(fields) for topic, fields in self.output_topic_subtopics.items()
        }
        self._original_input_non_numeric_topics = {
            topic: [dict(field) for field in fields]
            for topic, fields in self.input_non_numeric_topics.items()
        }
        self._original_output_non_numeric_topics = {
            topic: [dict(field) for field in fields]
            for topic, fields in self.output_non_numeric_topics.items()
        }

    def _normalize_topic_subtopics(
        self,
        value: object,
    ) -> dict[str, list[str]]:
        if not isinstance(value, Mapping):
            return {}
        return {
            str(topic): [str(field) for field in (fields or [])]
            for topic, fields in value.items()
            if isinstance(topic, str)
        }

    def _normalize_non_numeric_topics(
        self,
        value: object,
    ) -> dict[str, list[NonNumericTopicFieldSelection]]:
        if not isinstance(value, Mapping):
            return {}

        normalized: dict[str, list[NonNumericTopicFieldSelection]] = {}
        for topic, fields in value.items():
            if not isinstance(topic, str) or not isinstance(fields, list):
                continue

            normalized_fields: list[NonNumericTopicFieldSelection] = []
            for field in fields:
                if not isinstance(field, Mapping):
                    continue
                field_path = field.get("field_path")
                data_type = field.get("data_type")
                ros_type = field.get("ros_type")
                if (
                    not isinstance(field_path, str)
                    or not field_path
                    or data_type not in {"unordered_set", "image"}
                    or not isinstance(ros_type, str)
                    or not ros_type
                ):
                    continue
                normalized_fields.append(
                    {
                        "field_path": field_path,
                        "data_type": data_type,
                        "ros_type": ros_type,
                    },
                )

            if normalized_fields:
                normalized[topic] = normalized_fields

        return normalized

    def _normalize_non_numeric_topic_map(
        self,
        value: Mapping[str, list[NonNumericTopicFieldSelection]],
    ) -> dict[str, tuple[tuple[str, str, str], ...]]:
        normalized: dict[str, tuple[tuple[str, str, str], ...]] = {}
        for topic, fields in value.items():
            normalized[topic] = tuple(
                sorted(
                    (
                        field["field_path"],
                        field["data_type"],
                        field["ros_type"],
                    )
                    for field in fields
                ),
            )
        return normalized

    def _ordered_union(self, *topic_lists: list[str]) -> list[str]:
        seen: set[str] = set()
        ordered: list[str] = []
        for topic_list in topic_lists:
            for topic in topic_list:
                if topic in seen:
                    continue
                seen.add(topic)
                ordered.append(topic)
        return ordered

    def _on_home_title_click(self, _event: tk.Event | None = None) -> None:
        if self.controller is not None:
            self.controller.show_page("start")

    def _on_project_title_click(self, _event: tk.Event | None = None) -> None:
        if self.controller is not None:
            self.controller.show_page(
                "view_project",
                project={"robogym_project": {"name": self._current_project_name}},
            )

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
