import tkinter as tk

from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.get_gazebo_topics import get_gazebo_topics
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
        self.controller = controller

        self.world_control_client = WorldControlClient()

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

        tk.Button(
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
        ).grid(row=5, column=5, sticky="nsew", padx=(0, 14), pady=4)

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

        self.input_topic_var = tk.StringVar(
            value=self._topics[0] if self._topics else "",
        )
        self.output_topic_var = tk.StringVar(
            value=self._topics[0] if self._topics else "",
        )

        outer = tk.Frame(self, bg="#DADADA")
        outer.grid(row=7, column=0, columnspan=6, sticky="nsew", padx=14, pady=(0, 8))
        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)
        outer.grid_rowconfigure(0, weight=1)

        input_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        input_topic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        output_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        output_topic_frame.grid(row=0, column=1, sticky="nsew")

        if not self._topics:
            self._topics = ["No topics found"]

        for topic in self._topics:
            tk.Radiobutton(
                input_topic_frame.content,
                text=topic,
                variable=self.input_topic_var,
                value=topic,
                bg="#DADADA",
                fg="black",
                activebackground="#DADADA",
                font=("Arial", 14),
                anchor="w",
                highlightthickness=0,
            ).pack(anchor="w")

            tk.Radiobutton(
                output_topic_frame.content,
                text=topic,
                variable=self.output_topic_var,
                value=topic,
                bg="#DADADA",
                fg="black",
                activebackground="#DADADA",
                font=("Arial", 14),
                anchor="w",
                highlightthickness=0,
            ).pack(anchor="w")

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

        tk.Button(
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
        ).grid(row=8, column=3, columnspan=3, sticky="nsew", padx=(8, 14), pady=(8, 14))

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

    def _on_grab_from_sim(self):
        """
        Handle the "Grab from Sim" action.

        Current behavior is a placeholder that logs activation.
        """
        # Play the simulation
        self.world_control_client.play_simulation()

        # Enable keyboard controls

        # Display popups and wait for signal indicating both coordinates have been collected
        print("grab_from_sim_activation")

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

        Current behavior is a placeholder that logs activation.
        """
        print("create_project_activation")
