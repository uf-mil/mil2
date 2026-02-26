import tkinter as tk
from typing import Any


class ViewDemoPage(tk.Frame):
    """
    UI Page for viewing a demo from a project.
    """

    def __init__(self, parent: tk.Widget, controller: Any | None = None) -> None:
        super().__init__(parent, bg="#DADADA")
        self.controller = controller

        self._project_name = "Start Gate Agent"
        self._demo_name = "Demo 1"

        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self._title_row = tk.Frame(self, bg="#DADADA")
        self._title_row.grid(row=0, column=0, sticky="ew", padx=14, pady=(14, 8))
        self._title_row.grid_columnconfigure(0, weight=1)

        self._title_left = tk.Frame(self._title_row, bg="#DADADA")
        self._title_left.grid(row=0, column=0, sticky="w")

        self._home_title = tk.Label(
            self._title_left,
            text="MIL RoboGYM >",
            bg="#DADADA",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self._home_title.pack(side="left")
        self._home_title.bind("<Button-1>", self._on_home_title_click)

        self._project_title = tk.Label(
            self._title_left,
            text=f"{self._project_name} >",
            bg="#DADADA",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self._project_title.pack(side="left", padx=(6, 0))
        self._project_title.bind("<Button-1>", self._on_project_title_click)

        self._demo_title = tk.Label(
            self._title_left,
            text=self._demo_name,
            bg="#DADADA",
            font=("Arial", 20, "bold"),
        )
        self._demo_title.pack(side="left", padx=(6, 0))

        self._edit_btn = tk.Button(self._title_row, text="Edit", width=8)
        self._edit_btn.grid(row=0, column=1, sticky="e")

        self._subtitle = tk.Label(
            self,
            text="Sampling rate: 1 step / sec | World: src/default/world/file",
            bg="#DADADA",
            fg="#333333",
            font=("Arial", 11),
            anchor="w",
        )
        self._subtitle.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 8))

        self._content = tk.Frame(self, bg="#DADADA")
        self._content.grid(row=2, column=0, sticky="nsew", padx=14)
        self._content.grid_rowconfigure(0, weight=1)
        self._content.grid_columnconfigure(1, weight=1)

        self._steps_panel = tk.Frame(self._content, bg="#CFCFCF", width=260)
        self._steps_panel.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        self._steps_panel.grid_propagate(False)

        tk.Label(
            self._steps_panel,
            text="Steps",
            bg="#CFCFCF",
            font=("Arial", 12, "bold"),
            anchor="w",
        ).pack(fill="x", padx=8, pady=(8, 4))

        # Scrollable region
        self._steps_canvas = tk.Canvas(
            self._steps_panel,
            bg="#CFCFCF",
            highlightthickness=0,
        )
        self._steps_canvas.pack(side="left", fill="both", expand=True)

        scrollbar = tk.Scrollbar(self._steps_panel, command=self._steps_canvas.yview)
        scrollbar.pack(side="right", fill="y")

        self._steps_canvas.configure(yscrollcommand=scrollbar.set)

        self._steps_frame = tk.Frame(self._steps_canvas, bg="#CFCFCF")
        self._steps_canvas.create_window((0, 0), window=self._steps_frame, anchor="nw")

        self._steps_frame.bind(
            "<Configure>",
            lambda e: self._steps_canvas.configure(
                scrollregion=self._steps_canvas.bbox("all"),
            ),
        )

        self._data_panel = tk.Frame(self._content, bg="#CFCFCF")
        self._data_panel.grid(row=0, column=1, sticky="nsew")
        self._data_panel.grid_rowconfigure(1, weight=1)
        self._data_panel.grid_columnconfigure(0, weight=1)

        header = tk.Frame(self._data_panel, bg="#CFCFCF")
        header.grid(row=0, column=0, sticky="ew")

        tk.Label(
            header,
            text="Collected Data",
            bg="#CFCFCF",
            font=("Arial", 12, "bold"),
        ).pack(side="left", padx=8, pady=6)

        self._data_display = tk.Frame(
            self._data_panel,
            bg="#EAEAEA",
            bd=1,
            relief="sunken",
        )
        self._data_display.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))

        tk.Label(
            self._data_display,
            text="Table/Graph of collected input and output data",
            bg="#EAEAEA",
            fg="#555",
        ).place(relx=0.5, rely=0.5, anchor="center")

        self._controls = tk.Frame(self, bg="#CFCFCF", height=60)
        self._controls.grid(row=3, column=0, sticky="ew", padx=14, pady=(6, 14))
        self._controls.grid_propagate(False)

        btns = tk.Frame(self._controls, bg="#CFCFCF")
        btns.pack(pady=10)

        for label in ["Reset Demo", "Play/Continue", "Pause", "Undo", "Redo"]:
            tk.Button(btns, text=label, width=12).pack(side="left", padx=4)

        tk.Button(btns, text="Preposition", state="disabled", width=12).pack(
            side="left",
            padx=10,
        )
        tk.Button(btns, text="Rand. Pos.", state="disabled", width=12).pack(side="left")

        # Example data
        self._add_step_widget("Origin: (6, 7, 8, 0)")
        self._add_step_widget("(6,7,8,0)  →  (6,7.5,8,0)")
        self._add_step_widget("(6,7,8,0)  →  (6,7.5,8,0)", highlight=True)

    def _add_step_widget(self, text: str, highlight: bool = False) -> None:
        """Add a step entry to the left panel."""
        color = "#E6E6E6" if not highlight else "#CFE8FF"

        row = tk.Frame(self._steps_frame, bg=color, bd=1, relief="solid")
        row.pack(fill="x", padx=6, pady=4)

        tk.Label(row, text=text, bg=color, font=("Courier", 10)).pack(
            side="left",
            padx=6,
            pady=6,
        )

    def clear_steps(self) -> None:
        """Remove all steps."""
        for child in self._steps_frame.winfo_children():
            child.destroy()

    def _on_home_title_click(self, _event=None) -> None:
        if self.controller:
            self.controller.show_page("start")

    def _on_project_title_click(self, _event=None) -> None:
        if self.controller:
            self.controller.show_page("view_project")
