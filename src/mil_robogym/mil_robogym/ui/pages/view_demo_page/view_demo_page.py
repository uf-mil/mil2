import tkinter as tk
from typing import Any, Mapping

from .collected_data_section import CollectedDataSection
from .controls_section import ControlsSection
from .header import Header
from .steps_section import StepsSection


class ViewDemoPage(tk.Frame):
    """
    UI Page for viewing a demo from a project.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#DADADA")
        self.app = controller

        self._project: Mapping[str, Any] | None = None
        self._demo: Mapping[str, Any] | None = None

        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Header
        self.header = Header(
            self,
            on_home_click=self._on_home_click,
            on_project_click=self._on_project_click,
        )
        self.header.grid(row=0, column=0, sticky="ew", padx=14, pady=(14, 8))
        self.header.subtitle.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 8))

        # Content
        content = tk.Frame(self, bg="#DADADA")
        content.grid(row=2, column=0, sticky="nsew", padx=14)
        content.grid_rowconfigure(0, weight=1)
        content.grid_columnconfigure(1, weight=1)

        # Steps section
        self.steps = StepsSection(content)
        self.steps.grid(row=0, column=0, sticky="nsw", padx=(0, 10))

        # Data collected section
        self.data_section = CollectedDataSection(content)
        self.data_section.grid(row=0, column=1, sticky="nsew")

        # Controls
        self.controls = ControlsSection(self)
        self.controls.grid(row=3, column=0, sticky="ew", padx=14, pady=(6, 14))

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **kwargs: Any,
    ) -> None:
        self._project = project
        self._demo = kwargs.get("demo", {}).get("robogym_demo", {})

        demo_name = kwargs.get("demo_name", "")
        project_name = ""

        if project:
            project_name = project.get("robogym_project", {}).get("name", "")

        self.header.set_titles(project_name, demo_name)

        if self._demo and project:
            subtitle = (
                f"Sampling rate: {self._demo['sampling_rate']} steps / sec | "
                f"World: {project['robogym_project']['world_file']}"
            )
            self.header.set_subtitle(subtitle)

            self.steps.clear()
            self.steps.add_step(f"Origin: {self._demo['start_position']}")

    def _on_home_click(self) -> None:
        self.steps.clear()
        self.app.show_page("start")

    def _on_project_click(self) -> None:
        self.steps.clear()
        self.app.show_page("view_project", project=self._project)
