import tkinter as tk
from typing import Any

from .collected_data_section import CollectedDataSection
from .controls_section import ControlsSection
from .demo_view_controller import DemoViewController
from .header import Header
from .steps_section import StepsSection


class ViewDemoPage(tk.Frame):
    """
    UI Page for viewing a demo from a project.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#DADADA")
        self.controller = DemoViewController(self, controller)

        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Bindings
        self.bind("<space>", self.controller.pause_recording)

        # Header
        self.header = Header(self, self.controller)
        self.header.grid(row=0, column=0, sticky="ew", padx=14, pady=(14, 8))
        self.header.subtitle.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 8))

        # Content
        self.content = tk.Frame(self, bg="#DADADA")
        self.content.grid(row=2, column=0, sticky="nsew", padx=14)
        self.content.grid_rowconfigure(0, weight=1)
        self.content.grid_columnconfigure(1, weight=1)

        # Steps section
        self.steps = StepsSection(self.content, self.controller)
        self.steps.grid(row=0, column=0, sticky="nsw", padx=(0, 10))

        # Data collected section
        self.data_section = CollectedDataSection(self.content)
        self.data_section.grid(row=0, column=1, sticky="nsew")

        # Controls
        self.controls = ControlsSection(self, self.controller)
        self.controls.grid(row=3, column=0, sticky="ew", padx=14, pady=(6, 14))

    def set_context(self, project: dict[str, Any] | None = None, **kwargs: Any):
        self.controller.set_context(project, demo=kwargs.get("demo", {}))
