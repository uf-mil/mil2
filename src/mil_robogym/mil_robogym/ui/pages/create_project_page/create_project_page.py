import threading
import tkinter as tk

from mil_robogym.clients.model_pose_client import ModelPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.build_tensor_spec import build_tensor_spec
from mil_robogym.data_collection.filesystem import (
    create_project_folder,
)
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.data_collection.get_ros2_topics import get_ros2_topics
from mil_robogym.data_collection.sample_input_topics import sample_topics
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls import TeleopGUI
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame


class CreateProjectPage(tk.Frame):

    def __init__(self, parent, controller=None):
        """
        Build and lay out the "Create Project" page.

        Initializes the page state, fetches ROS 2-derived defaults, and creates
        all widgets used to configure and submit a project definition.

        :param parent: Parent Tkinter widget that owns this frame.
        :param controller: Optional page controller that supports `show_page`.
            Used by the cancel action to navigate back to the start page.
        :type controller: object | None
        """
        super().__init__(parent, bg="#DADADA")

        self.coordinate1 = None
        self.coordinate2 = None

        self.controller = controller

        self.world_control_client = WorldControlClient()
        self.gz_pose_client = ModelPoseClient()

        self.keyboard_controls_gui = None
        self.popup = None
        self._create_project_error_tip = None
        self._create_project_error_tip_after_id = None
        self._tensor_spec = None
        self._tensor_spec_topic_signature = None

        self._topics = self._safe_get_topics()
        self._world_default = self._safe_get_world_file()

        title_row = tk.Frame(self, bg="#DADADA")
        title_row.grid(row=0, column=0, columnspan=6, sticky="w", padx=14, pady=(14, 8))

        home_title = tk.Label(
            title_row,
            text="MIL Robogym >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        home_title.pack(side="left")
        home_title.configure(cursor="hand2")
        home_title.bind("<Button-1>", self._on_home_title_click)

        page_title = tk.Label(
            title_row,
            text="Create Project",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            anchor="w",
        )
        page_title.pack(side="left", padx=(6, 0))

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
        self.project_name_var.trace_add(
            "write",
            lambda *_: self._update_create_project_button_state(),
        )
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
        self.coord1_var.trace_add(
            "write",
            lambda *_: self._update_create_project_button_state(),
        )
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
        self.coord2_var.trace_add(
            "write",
            lambda *_: self._update_create_project_button_state(),
        )
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

        self.input_topic_vars = {}
        self.output_topic_vars = {}
        self.input_topic_buttons = {}
        self.output_topic_buttons = {}
        self.input_subtopic_vars = {}
        self.output_subtopic_vars = {}
        self.input_subtopic_field_order = {}
        self.output_subtopic_field_order = {}
        self.input_subtopic_numeric_fields = {}
        self.output_subtopic_numeric_fields = {}
        self.input_subtopic_expanded = {}
        self.output_subtopic_expanded = {}
        self.input_subtopic_loading = set()
        self.output_subtopic_loading = set()
        self.input_subtopic_load_errors = {}
        self.output_subtopic_load_errors = {}
        self.input_subtopic_summary_labels = {}
        self.output_subtopic_summary_labels = {}

        outer = tk.Frame(self, bg="#DADADA")
        outer.grid(row=7, column=0, columnspan=6, sticky="nsew", padx=14, pady=(0, 8))
        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)
        outer.grid_rowconfigure(0, weight=1)

        self.input_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        self.input_topic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        self.output_topic_frame.grid(row=0, column=1, sticky="nsew")

        if not self._topics:
            self._topics = ["No topics found"]

        self.input_topic_order = list(self._topics)
        self.output_topic_order = list(self._topics)
        self._build_topic_checkboxes(list_type="input")
        self._build_topic_checkboxes(list_type="output")

        tk.Label(
            self,
            text="Input Subtopics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15, "bold"),
            anchor="w",
        ).grid(
            row=8,
            column=0,
            columnspan=2,
            sticky="w",
            padx=14,
            pady=(0, 2),
        )
        tk.Label(
            self,
            text="Output Subtopics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15, "bold"),
            anchor="w",
        ).grid(
            row=8,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(8, 14),
            pady=(0, 2),
        )

        subtopic_outer = tk.Frame(self, bg="#DADADA")
        subtopic_outer.grid(
            row=9,
            column=0,
            columnspan=6,
            sticky="nsew",
            padx=14,
            pady=(0, 8),
        )
        subtopic_outer.grid_columnconfigure(0, weight=1)
        subtopic_outer.grid_columnconfigure(1, weight=1)
        subtopic_outer.grid_rowconfigure(0, weight=1)

        self.input_subtopic_frame = ScrollableFrame(subtopic_outer, bg="#DADADA")
        self.input_subtopic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_subtopic_frame = ScrollableFrame(subtopic_outer, bg="#DADADA")
        self.output_subtopic_frame.grid(row=0, column=1, sticky="nsew")

        self._refresh_subtopic_sections("input")
        self._refresh_subtopic_sections("output")

        self.compute_tensor_spec_button = tk.Button(
            self,
            text="Compute Tensor Spec",
            command=self._on_compute_tensor_spec,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 14),
            padx=8,
            pady=5,
        )
        self.compute_tensor_spec_button.grid(
            row=10,
            column=0,
            columnspan=2,
            sticky="nsew",
            padx=(14, 8),
            pady=(2, 6),
        )

        self.tensor_spec_status_var = tk.StringVar(value="Tensor Spec: Not computed")
        self.tensor_spec_status_label = tk.Label(
            self,
            textvariable=self.tensor_spec_status_var,
            bg="#DADADA",
            fg="#444444",
            font=("Arial", 12),
            anchor="w",
        )
        self.tensor_spec_status_label.grid(
            row=10,
            column=2,
            columnspan=4,
            sticky="nsew",
            padx=(8, 14),
            pady=(2, 6),
        )

        action_row = 11

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
        ).grid(
            row=action_row,
            column=0,
            columnspan=3,
            sticky="nsew",
            padx=(14, 8),
            pady=(8, 14),
        )

        self.create_project_button = tk.Button(
            self,
            text="Create Project",
            command=self._on_create_project,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 16),
            padx=10,
            pady=6,
        )
        self.create_project_button.grid(
            row=action_row,
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
        self._update_create_project_button_state()

    def _safe_get_topics(self):
        """
        Retrieve available ROS 2 topics with normalized error handling.

        :raises RuntimeError: If topic discovery fails for any supported
            ROS 2/runtime error condition.
        :return: List of discovered topic names.
        :rtype: list[str]
        """
        try:
            return get_ros2_topics()
        except (RuntimeError, FileNotFoundError) as e:
            raise RuntimeError(
                "Getting ROS 2 topics on create projects page failed",
            ) from e

    def _safe_get_world_file(self):
        """
        For now, this will return the subjugator bringup Gazebo world file.
        This will be hardcoded until can be changed otherwise to include other world file

        :return: Absolute or expanded path to the robosub_2025.world file
        :rtype: str
        """
        return "~/mil2/install/subjugator_gazebo/share/subjugator_gazebo/worlds/robosub_2025.world"

    def _toggle_random_spawn(self):
        """
        Enable or disable random-spawn coordinate inputs.

        Mirrors the random spawn checkbox state to the coordinate entry widgets
        and updates label colors to indicate enabled versus disabled state.
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
        self._update_create_project_button_state()

    def _build_topic_checkboxes(self, list_type):
        """
        Build topic checkboxes for either input or output topics.
        """
        if list_type == "input":
            frame = self.input_topic_frame.content
            topic_order = self.input_topic_order
            topic_vars = self.input_topic_vars
            topic_buttons = self.input_topic_buttons
        else:
            frame = self.output_topic_frame.content
            topic_order = self.output_topic_order
            topic_vars = self.output_topic_vars
            topic_buttons = self.output_topic_buttons

        for topic in topic_order:
            topic_var = topic_vars.get(topic, tk.BooleanVar(value=False))
            topic_vars[topic] = topic_var
            topic_button = tk.Checkbutton(
                frame,
                text=topic,
                variable=topic_var,
                command=lambda t=topic, lt=list_type: self._on_topic_clicked(t, lt),
                bg="#DADADA",
                fg="black",
                activebackground="#DADADA",
                font=("Arial", 14),
                anchor="w",
                highlightthickness=0,
            )
            topic_button.pack(anchor="w")
            topic_buttons[topic] = topic_button

    def _on_topic_clicked(self, topic, list_type):
        """
        Move the clicked topic to the top of only its own topic list.
        """
        if list_type == "input":
            topic_order = self.input_topic_order
            topic_buttons = self.input_topic_buttons
        else:
            topic_order = self.output_topic_order
            topic_buttons = self.output_topic_buttons

        if topic not in topic_order:
            return

        topic_order.remove(topic)
        topic_order.insert(0, topic)

        for ordered_topic in topic_order:
            button = topic_buttons[ordered_topic]
            button.pack_forget()
            button.pack(anchor="w")
        self._invalidate_tensor_spec()
        self._refresh_subtopic_sections(list_type)
        self._update_create_project_button_state()

    def _subtopic_state(self, list_type):
        """
        Return subtopic state containers for one topic list type.
        """
        if list_type == "input":
            return {
                "frame": self.input_subtopic_frame.content,
                "expanded": self.input_subtopic_expanded,
                "field_order": self.input_subtopic_field_order,
                "numeric_fields": self.input_subtopic_numeric_fields,
                "vars": self.input_subtopic_vars,
                "loading": self.input_subtopic_loading,
                "load_errors": self.input_subtopic_load_errors,
                "summary_labels": self.input_subtopic_summary_labels,
            }
        return {
            "frame": self.output_subtopic_frame.content,
            "expanded": self.output_subtopic_expanded,
            "field_order": self.output_subtopic_field_order,
            "numeric_fields": self.output_subtopic_numeric_fields,
            "vars": self.output_subtopic_vars,
            "loading": self.output_subtopic_loading,
            "load_errors": self.output_subtopic_load_errors,
            "summary_labels": self.output_subtopic_summary_labels,
        }

    def _is_numeric_subtopic_value(self, value: object) -> bool:
        """
        Return True when a sampled subtopic value maps to a scalar tensor value.
        """
        return isinstance(value, (bool, int, float))

    def _sample_subtopics_for_topic(
        self,
        topic: str,
    ) -> tuple[list[str], set[str], str | None]:
        """
        Sample one topic and return `(field_order, numeric_fields, error_text)`.
        """
        try:
            sampled = sample_topics([topic])
            flattened = sampled[topic]
        except (RuntimeError, ValueError, KeyError) as e:
            return [], set(), str(e) or type(e).__name__

        field_order = list(flattened.keys())
        numeric_fields = {
            field
            for field, value in flattened.items()
            if self._is_numeric_subtopic_value(value)
        }
        return field_order, numeric_fields, None

    def _start_subtopic_load(self, topic: str, list_type: str) -> None:
        """
        Start async subtopic loading for one topic if not already loaded/loading.
        """
        state = self._subtopic_state(list_type)
        if topic in state["field_order"] or topic in state["loading"]:
            return

        state["loading"].add(topic)
        state["load_errors"].pop(topic, None)

        def _worker() -> None:
            field_order, numeric_fields, error_text = self._sample_subtopics_for_topic(
                topic,
            )
            self.after(
                0,
                lambda: self._finish_subtopic_load(
                    topic,
                    list_type,
                    field_order,
                    numeric_fields,
                    error_text,
                ),
            )

        threading.Thread(target=_worker, daemon=True).start()

    def _finish_subtopic_load(
        self,
        topic: str,
        list_type: str,
        field_order: list[str],
        numeric_fields: set[str],
        error_text: str | None,
    ) -> None:
        """
        Apply async subtopic loading results on the Tk main thread.
        """
        state = self._subtopic_state(list_type)
        state["loading"].discard(topic)

        if error_text is not None:
            state["load_errors"][topic] = error_text
            self._refresh_subtopic_sections(list_type)
            return

        state["load_errors"].pop(topic, None)
        state["field_order"][topic] = field_order
        state["numeric_fields"][topic] = numeric_fields

        topic_vars = state["vars"].setdefault(topic, {})
        for field in field_order:
            if field in numeric_fields and field not in topic_vars:
                topic_vars[field] = tk.BooleanVar(value=True)

        self._refresh_subtopic_sections(list_type)

    def _subtopic_summary_text(self, topic: str, list_type: str) -> str:
        """
        Build summary text for one topic's subtopic selection state.
        """
        state = self._subtopic_state(list_type)
        field_order = state["field_order"].get(topic, [])
        numeric_fields = state["numeric_fields"].get(topic, set())
        topic_vars = state["vars"].get(topic, {})
        is_loading = topic in state["loading"]
        load_error = state["load_errors"].get(topic)

        if is_loading and not field_order:
            return f"{topic} (loading subtopics...)"
        if load_error and not field_order:
            return f"{topic} (failed to load subtopics)"

        selected_numeric = sum(
            1
            for field in field_order
            if field in numeric_fields
            and (field not in topic_vars or topic_vars[field].get())
        )
        non_numeric_count = len(field_order) - len(numeric_fields)

        summary = f"{topic} ({selected_numeric}/{len(numeric_fields)} numeric selected"
        if non_numeric_count:
            summary += f", {non_numeric_count} non-numeric"
        summary += ")"
        return summary

    def _update_subtopic_summary(self, topic: str, list_type: str) -> None:
        """
        Update the rendered summary label text for one topic, if visible.
        """
        state = self._subtopic_state(list_type)
        label = state["summary_labels"].get(topic)
        if label is None:
            return
        label.configure(text=self._subtopic_summary_text(topic, list_type))

    def _toggle_subtopic_section(self, topic: str, list_type: str) -> None:
        """
        Expand or collapse one topic's subtopic list.
        """
        state = self._subtopic_state(list_type)
        state["expanded"][topic] = not state["expanded"].get(topic, False)
        self._refresh_subtopic_sections(list_type)

    def _on_subtopic_toggled(self, topic: str, list_type: str) -> None:
        """
        Invalidate cached tensor spec after changing selected subtopics.
        """
        self._invalidate_tensor_spec()
        self._update_subtopic_summary(topic, list_type)

    def _refresh_subtopic_sections(self, list_type: str) -> None:
        """
        Rebuild the subtopic panel for selected topics in one list.
        """
        state = self._subtopic_state(list_type)
        frame = state["frame"]
        state["summary_labels"].clear()
        for child in frame.winfo_children():
            child.destroy()

        selected_topics = self._selected_topics(list_type)
        if not selected_topics:
            tk.Label(
                frame,
                text="Select a topic to expand subtopics.",
                bg="#DADADA",
                fg="#666666",
                font=("Arial", 11),
                anchor="w",
            ).pack(anchor="w")
            return

        for topic in selected_topics:
            section = tk.Frame(frame, bg="#DADADA", bd=1, relief="flat")
            section.pack(fill="x", anchor="w", pady=(0, 6))

            if topic not in state["field_order"]:
                self._start_subtopic_load(topic, list_type)

            header = tk.Frame(section, bg="#DADADA")
            header.pack(fill="x")

            is_expanded = state["expanded"].get(topic, False)
            toggle = tk.Button(
                header,
                text="▼" if is_expanded else "▶",
                command=lambda t=topic, lt=list_type: self._toggle_subtopic_section(
                    t,
                    lt,
                ),
                bg="#ECECEC",
                activebackground="#DFDFDF",
                fg="black",
                relief="solid",
                bd=1,
                font=("Arial", 9, "bold"),
                width=2,
                padx=0,
                pady=0,
            )
            toggle.pack(side="left", padx=(0, 6))

            summary_label = tk.Label(
                header,
                text=self._subtopic_summary_text(topic, list_type),
                bg="#DADADA",
                fg="black",
                font=("Arial", 11),
                anchor="w",
            )
            summary_label.pack(side="left", fill="x", expand=True)
            state["summary_labels"][topic] = summary_label

            if not is_expanded:
                continue

            fields_container = tk.Frame(section, bg="#DADADA")
            fields_container.pack(fill="x", padx=(24, 0), pady=(4, 0))

            field_order = state["field_order"].get(topic, [])
            is_loading = topic in state["loading"]
            load_error = state["load_errors"].get(topic)
            if load_error is not None:
                tk.Label(
                    fields_container,
                    text=f"Failed to load subtopics: {load_error}",
                    bg="#DADADA",
                    fg="#AA2222",
                    font=("Arial", 10),
                    anchor="w",
                ).pack(anchor="w")
                continue

            if is_loading and not field_order:
                tk.Label(
                    fields_container,
                    text="Loading subtopics...",
                    bg="#DADADA",
                    fg="#666666",
                    font=("Arial", 10),
                    anchor="w",
                ).pack(anchor="w")
                continue

            numeric_fields = state["numeric_fields"].get(topic, set())
            topic_vars = state["vars"].setdefault(topic, {})
            if not field_order:
                tk.Label(
                    fields_container,
                    text="No subtopics available.",
                    bg="#DADADA",
                    fg="#666666",
                    font=("Arial", 10),
                    anchor="w",
                ).pack(anchor="w")
                continue

            for field in field_order:
                if field in numeric_fields:
                    field_var = topic_vars.setdefault(field, tk.BooleanVar(value=True))
                    tk.Checkbutton(
                        fields_container,
                        text=field,
                        variable=field_var,
                        command=lambda t=topic, lt=list_type: self._on_subtopic_toggled(
                            t,
                            lt,
                        ),
                        bg="#DADADA",
                        fg="black",
                        activebackground="#DADADA",
                        font=("Arial", 10),
                        anchor="w",
                        highlightthickness=0,
                    ).pack(anchor="w")
                    continue

                tk.Label(
                    fields_container,
                    text=f"{field} (non-numeric)",
                    bg="#DADADA",
                    fg="#777777",
                    font=("Arial", 10),
                    anchor="w",
                ).pack(anchor="w")

    def _selected_topics(self, list_type):
        """
        Return selected topics in current UI order for one topic list.
        """
        if list_type == "input":
            topic_order = self.input_topic_order
            topic_vars = self.input_topic_vars
        else:
            topic_order = self.output_topic_order
            topic_vars = self.output_topic_vars

        return [topic for topic in topic_order if topic_vars[topic].get()]

    def _current_tensor_spec_signature(self):
        """
        Return the selected input/output topics as a cache signature.
        """
        return (
            tuple(self._selected_topics("input")),
            tuple(self._selected_topics("output")),
        )

    def _selected_subtopic_fields(self, list_type: str) -> dict[str, set[str]]:
        """
        Return selected numeric subtopic fields per selected topic.
        """
        state = self._subtopic_state(list_type)
        selected_fields: dict[str, set[str]] = {}
        for topic in self._selected_topics(list_type):
            field_order = state["field_order"].get(topic)
            numeric_fields = state["numeric_fields"].get(topic)
            topic_vars = state["vars"].get(topic, {})
            if not field_order or numeric_fields is None:
                continue

            chosen = {
                field
                for field in field_order
                if field in numeric_fields
                and (field not in topic_vars or topic_vars[field].get())
            }
            selected_fields[topic] = chosen
        return selected_fields

    def _apply_subtopic_filters_to_tensor_spec(self, tensor_spec: dict) -> dict:
        """
        Apply selected numeric subtopic filters to tensor spec features.
        """
        filtered = dict(tensor_spec)
        input_filters = self._selected_subtopic_fields("input")
        output_filters = self._selected_subtopic_fields("output")

        def _feature_kept(feature: str, allowed_fields: dict[str, set[str]]) -> bool:
            if ":" not in feature:
                return True
            topic, field = feature.split(":", 1)
            allowed = allowed_fields.get(topic)
            if allowed is None:
                return True
            return field in allowed

        if input_filters:
            filtered["input_features"] = [
                feature
                for feature in tensor_spec["input_features"]
                if _feature_kept(feature, input_filters)
            ]
            filtered["input_dim"] = len(filtered["input_features"])

        if output_filters:
            filtered["output_features"] = [
                feature
                for feature in tensor_spec["output_features"]
                if _feature_kept(feature, output_filters)
            ]
            filtered["output_dim"] = len(filtered["output_features"])

        return filtered

    def _set_tensor_spec_status(self, text: str, *, fg: str) -> None:
        """
        Set the tensor spec status text and color in the UI.
        """
        self.tensor_spec_status_var.set(text)
        if self.tensor_spec_status_label is not None:
            self.tensor_spec_status_label.configure(fg=fg)

    def _invalidate_tensor_spec(self) -> None:
        """
        Clear cached tensor spec when selected topics change.
        """
        self._tensor_spec = None
        self._tensor_spec_topic_signature = None
        self._set_tensor_spec_status("Tensor Spec: Not computed", fg="#444444")

    def _build_project_config(self):
        """
        Build a project config payload from current form values.
        """
        random_spawn_enabled = self.random_spawn_var.get()
        coord1 = self.coordinate1 or self._parse_coord(self.coord1_var.get())
        coord2 = self.coordinate2 or self._parse_coord(self.coord2_var.get())

        return {
            "project_name": self.project_name_var.get().strip(),
            "world_file": self.world_file_var.get().strip(),
            "model_name": self.model_name_var.get().strip(),
            "random_spawn_space": {
                "enabled": random_spawn_enabled,
                "coord1_4d": coord1 or (0.0, 0.0, 0.0, 0.0),
                "coord2_4d": coord2 or (0.0, 0.0, 0.0, 0.0),
            },
            "input_topics": self._selected_topics("input"),
            "output_topics": self._selected_topics("output"),
        }

    def _on_compute_tensor_spec(self):
        """
        Compute tensor dimensions from current topic selections.
        """
        selected_input_topics = self._selected_topics("input")
        selected_output_topics = self._selected_topics("output")
        if not selected_input_topics or not selected_output_topics:
            self._invalidate_tensor_spec()
            self._show_create_project_error_tooltip(
                "Select at least one input and output topic before computing tensor spec.",
            )
            return

        project_cfg = self._build_project_config()

        try:
            tensor_spec = build_tensor_spec(project_cfg)
        except (RuntimeError, ValueError, KeyError) as e:
            self._invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_create_project_error_tooltip(str(e) or type(e).__name__)
            return

        tensor_spec = self._apply_subtopic_filters_to_tensor_spec(tensor_spec)
        if selected_input_topics and tensor_spec["input_dim"] == 0:
            self._invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_create_project_error_tooltip(
                "No numeric input subtopics selected. Expand input topics and select at least one numeric subtopic.",
            )
            return

        if selected_output_topics and tensor_spec["output_dim"] == 0:
            self._invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_create_project_error_tooltip(
                "No numeric output subtopics selected. Expand output topics and select at least one numeric subtopic.",
            )
            return

        self._tensor_spec = tensor_spec
        self._tensor_spec_topic_signature = self._current_tensor_spec_signature()
        ignored_input = sum(
            len(fields) for fields in tensor_spec["ignored_input_features"].values()
        )
        ignored_output = sum(
            len(fields) for fields in tensor_spec["ignored_output_features"].values()
        )
        self._set_tensor_spec_status(
            (
                "Tensor Spec: "
                f"in={tensor_spec['input_dim']} "
                f"out={tensor_spec['output_dim']} "
                f"ignored_in={ignored_input} "
                f"ignored_out={ignored_output}"
            ),
            fg="#222222",
        )
        self._hide_create_project_error_tooltip()

    def _on_grab_from_sim(self):
        """
        Handle the "Grab from Sim" action.

        Current behavior is a placeholder that logs activation.
        """
        # Play the simulation
        self.world_control_client.play_simulation()

        # Run keyboard controls
        self.keyboard_controls_gui = self.keyboard_controls_gui or TeleopGUI(
            self,
            self._on_close_of_keyboard_controls,
        )
        self.keyboard_controls_gui.show()

        # Display popups and wait for signal indicating both coordinates have been collected
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

    def _display_collected_coords(self, c1, c2):
        """
        Display the coordinates collected from simulation.
        """
        if c1 and c2:
            self.coordinate1 = c1
            self.coord1_entry.delete(0, tk.END)
            self.coord1_entry.insert(0, f"{tuple(round(v, 1) for v in c1)}")

            self.coordinate2 = c2
            self.coord2_entry.delete(0, tk.END)
            self.coord2_entry.insert(0, f"{tuple(round(v, 1) for v in c2)}")

        self.keyboard_controls_gui.hide()
        self.world_control_client.pause_simulation()
        self.popup = None
        self._update_create_project_button_state()

    def _on_close_of_keyboard_controls(self):
        """
        Close "Grabbing Coordinates From Simulation" window.
        """
        self.popup.finish()

    def _on_cancel(self):
        """
        Handle cancel action by returning to the start page when available.

        If a page controller was provided at initialization time, this invokes
        `controller.show_page("start")`.
        """
        if self.controller is not None:
            self.controller.show_page("start")

    def _on_home_title_click(self, _event):
        """
        Handle clicking the home breadcrumb label.
        """
        if self.controller is not None:
            self.controller.show_page("start")

    def _on_create_project(self):
        """
        Handle the "Create Project" action.
        """
        if not self._is_form_valid():
            self._update_create_project_button_state()
            return

        self._update_create_project_button_state()
        project_cfg = self._build_project_config()
        project_name = project_cfg["project_name"]

        if (
            self._tensor_spec is not None
            and self._tensor_spec_topic_signature
            == self._current_tensor_spec_signature()
        ):
            project_cfg["tensor_spec"] = self._tensor_spec

        try:
            create_project_folder(project_cfg)
        except (FileExistsError, OSError, RuntimeError, ValueError, KeyError) as e:
            self._set_create_project_button_enabled(False)
            self._show_create_project_error_tooltip(str(e) or type(e).__name__)
            return

        if self.controller is not None:
            project_payload = None
            for project in get_all_project_config():
                if project.get("robogym_project", {}).get("name") == project_name:
                    project_payload = project
                    break

            if project_payload is None:
                project_payload = {
                    "robogym_project": {"name": project_name},
                    "num_demos": 1,
                }

            self.controller.show_page("view_project", project=project_payload)

    def _is_filesystem_safe_name(self, name: str) -> bool:
        if not name.strip():
            return False
        forbidden = set('/\\:*?"<>|')
        return not any(ch in forbidden for ch in name)

    def _parse_coord(self, value: str):
        cleaned = value.strip().strip("()")
        parts = [p.strip() for p in cleaned.split(",")]
        if len(parts) != 4:
            return None
        try:
            return tuple(float(p) for p in parts)
        except ValueError:
            return None

    def _is_form_valid(self) -> bool:
        project_name = self.project_name_var.get().strip()
        selected_input_topics = any(
            self.input_topic_vars[topic].get() for topic in self.input_topic_order
        )
        selected_output_topics = any(
            self.output_topic_vars[topic].get() for topic in self.output_topic_order
        )
        random_spawn_enabled = self.random_spawn_var.get()

        coord1 = self.coordinate1 or self._parse_coord(self.coord1_var.get())
        coord2 = self.coordinate2 or self._parse_coord(self.coord2_var.get())

        has_valid_name = self._is_filesystem_safe_name(project_name)
        has_valid_topics = selected_input_topics and selected_output_topics
        has_valid_coords = (not random_spawn_enabled) or (
            coord1 is not None and coord2 is not None
        )
        return has_valid_name and has_valid_topics and has_valid_coords

    def _set_create_project_button_enabled(self, enabled: bool) -> None:
        if enabled:
            self.create_project_button.configure(
                state=tk.NORMAL,
                text="Create Project",
                fg="black",
                bg="#ECECEC",
                activebackground="#DFDFDF",
            )
            self._hide_create_project_error_tooltip()
            return

        self.create_project_button.configure(
            state=tk.DISABLED,
            text="Create Project",
            fg="#888888",
            bg="#E0E0E0",
            activebackground="#E0E0E0",
            disabledforeground="#888888",
        )
        self._hide_create_project_error_tooltip()

    def _update_create_project_button_state(self) -> None:
        self._set_create_project_button_enabled(self._is_form_valid())

    def _show_create_project_error_tooltip(self, message: str) -> None:
        self._hide_create_project_error_tooltip()

        tip = tk.Toplevel(self)
        tip.wm_overrideredirect(True)
        tip.attributes("-topmost", True)

        x = self.create_project_button.winfo_rootx()
        y = self.create_project_button.winfo_rooty() - 30
        tip.geometry(f"+{x}+{y}")

        tk.Label(
            tip,
            text=message,
            bg="#2B2B2B",
            fg="white",
            font=("Arial", 10),
            padx=8,
            pady=4,
        ).pack()

        self._create_project_error_tip = tip
        self._create_project_error_tip_after_id = self.after(
            4000,
            self._hide_create_project_error_tooltip,
        )

    def _hide_create_project_error_tooltip(self) -> None:
        if self._create_project_error_tip_after_id is not None:
            self.after_cancel(self._create_project_error_tip_after_id)
            self._create_project_error_tip_after_id = None

        if self._create_project_error_tip is not None:
            self._create_project_error_tip.destroy()
            self._create_project_error_tip = None
