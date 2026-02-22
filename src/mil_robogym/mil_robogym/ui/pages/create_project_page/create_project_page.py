import tkinter as tk

from mil_robogym.clients.model_pose_client import ModelPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.filesystem import (
    create_demo_folder,
    create_project_folder,
)
from mil_robogym.data_collection.get_gazebo_topics import get_gazebo_topics
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls import TeleopGUI
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame


class CreateProjectPage(tk.Frame):
    def __init__(self, parent, controller=None):
        """
        Build and lay out the "Create Project" page.

        Initializes the page state, fetches Gazebo-derived defaults, and creates
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

        self.input_topic_vars = {}
        self.output_topic_vars = {}
        self.input_topic_buttons = {}
        self.output_topic_buttons = {}

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
        ).grid(row=8, column=0, columnspan=3, sticky="nsew", padx=(14, 8), pady=(8, 14))

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

    def _safe_get_topics(self):
        """
        Retrieve available Gazebo topics with normalized error handling.

        :raises RuntimeError: If topic discovery fails for any supported
            Gazebo/runtime error condition.
        :return: List of discovered topic names.
        :rtype: list[str]
        """
        try:
            return get_gazebo_topics()
        except (RuntimeError, FileNotFoundError) as e:
            raise RuntimeError(
                "Getting gazebo topics on create projects page failed",
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

        def _is_filesystem_safe_name(name: str) -> bool:
            if not name.strip():
                return False
            forbidden = set('/\\:*?"<>|')
            return not any(ch in forbidden for ch in name)

        def _parse_coord(value: str):
            cleaned = value.strip().strip("()")
            parts = [p.strip() for p in cleaned.split(",")]
            if len(parts) != 4:
                return None
            try:
                return tuple(float(p) for p in parts)
            except ValueError:
                return None

        project_name = self.project_name_var.get().strip()
        selected_input_topics = [
            topic
            for topic in self.input_topic_order
            if self.input_topic_vars[topic].get()
        ]
        selected_output_topics = [
            topic
            for topic in self.output_topic_order
            if self.output_topic_vars[topic].get()
        ]
        random_spawn_enabled = self.random_spawn_var.get()

        coord1 = self.coordinate1 or _parse_coord(self.coord1_var.get())
        coord2 = self.coordinate2 or _parse_coord(self.coord2_var.get())

        has_invalid_name = not _is_filesystem_safe_name(project_name)
        has_missing_topics = not selected_input_topics or not selected_output_topics
        has_missing_coords = random_spawn_enabled and (coord1 is None or coord2 is None)

        if has_invalid_name or has_missing_topics or has_missing_coords:
            self.create_project_button.configure(text="✖ Create Project", fg="#C62828")
            return

        self.create_project_button.configure(text="Create Project", fg="black")

        project_cfg = {
            "project_name": project_name,
            "world_file": self.world_file_var.get().strip(),
            "model_name": self.model_name_var.get().strip(),
            "random_spawn_space": {
                "enabled": random_spawn_enabled,
                "coord1_4d": coord1 or (0.0, 0.0, 0.0, 0.0),
                "coord2_4d": coord2 or (0.0, 0.0, 0.0, 0.0),
            },
            "input_topics": selected_input_topics,
            "output_topics": selected_output_topics,
        }

        try:
            project_dir = create_project_folder(project_cfg)
            create_demo_folder(project_dir, demo_name="Demo 1", sampling_rate=1.0)
        except (FileExistsError, OSError, RuntimeError, ValueError, KeyError):
            self.create_project_button.configure(text="✖ Create Project", fg="#C62828")
            return

        if self.controller is not None:
            self.controller.show_page("start")
