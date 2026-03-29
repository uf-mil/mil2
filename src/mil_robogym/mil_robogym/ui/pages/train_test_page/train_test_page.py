from __future__ import annotations

import tkinter as tk
from pathlib import Path
from typing import Any, Mapping

from mil_robogym.data_collection.get_all_project_config import find_projects_dir
from mil_robogym.data_collection.utils import to_lower_snake_case

from .buttons_section import ButtonsSection
from .header_section import HeaderSection
from .history_section import HistorySection
from .metrics_section import MetricsSection
from .terminal_section import TerminalSection
from .train_test_controller import TrainTestViewController


class TrainTestPage(tk.Frame):
    """Train/Test page for selecting agents, viewing metrics, and running actions."""

    def __init__(self, parent: tk.Widget, controller: Any | None = None) -> None:
        super().__init__(parent, bg="#DADADA")
        self.controller = TrainTestViewController(self, controller)

        self.project: Mapping[str, Any] | None = None
        self.project_name = "Project"
        self.project_dir: Path | None = None
        self.selected_agent_name: str | None = None
        self._following_live_metrics = False

        self.header_section = HeaderSection(
            self,
            self.controller.navigate_to_home,
            self.controller.navigate_to_project,
            self.controller.navigate_to_settings,
        )
        self.history_section = HistorySection(
            self,
            self._on_agent_row_click,
            self._on_download_click,
        )
        self.metrics_section = MetricsSection(self)
        self.terminal_section = TerminalSection(self, "")
        self.buttons_section = ButtonsSection(
            self,
            self.controller.start_training,
            self._on_test_selected_agent_click,
        )

        for col in range(6):
            self.grid_columnconfigure(col, weight=1, uniform="half")
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """Set selected project context and refresh visible sections."""
        self.project = project
        self.project_name = (
            project.get("robogym_project", {}).get("name", "Project")
            if project is not None
            else "Project"
        )
        self.header_section.set_project_name(self.project_name)

        self.project_dir = None
        self.selected_agent_name = None
        self._following_live_metrics = False
        if project is not None:
            projects_dir = find_projects_dir()
            self.project_dir = projects_dir / to_lower_snake_case(self.project_name)

        restored_state = self._restore_ui_state()
        preferred_agent_name = restored_state.get("selected_agent_name")
        selected_metrics = restored_state.get("selected_metrics")
        if isinstance(selected_metrics, list):
            self.metrics_section.set_selected_metrics(
                [str(metric_name) for metric_name in selected_metrics],
            )

        self._refresh_history(
            preferred_agent_name if isinstance(preferred_agent_name, str) else None,
        )
        self._load_selected_agent_metrics()

        self.controller.set_context(project)

    def _refresh_history(
        self,
        preferred_agent_name: str | None = None,
        *,
        preserve_selection: bool = True,
    ) -> None:
        """Reload the saved-agent history without overriding live metrics mode."""

        if self.project_dir is None:
            for child in self.history_section.rows_frame.winfo_children():
                child.destroy()
            self.header_section.set_last_training_session(None)
            self.selected_agent_name = None
            return

        agent_names = self.history_section.load_agents(self.project_dir)
        selected_agent = None
        if preferred_agent_name in agent_names:
            selected_agent = preferred_agent_name
        elif preserve_selection and self.selected_agent_name in agent_names:
            selected_agent = self.selected_agent_name
        if selected_agent is None:
            selected_agent = agent_names[0] if agent_names else None

        self.selected_agent_name = selected_agent
        self.header_section.set_last_training_session(selected_agent)

    def _load_selected_agent_metrics(self) -> None:
        """Render the currently selected saved agent from CSV."""
        if self._following_live_metrics:
            return
        self.metrics_section.load_metrics_csv(self._get_selected_metrics_csv_path())

    def refresh_project_artifacts(
        self,
        preferred_agent_name: str | None = None,
    ) -> None:
        """Reload history and switch to a saved agent after training finishes."""
        self._following_live_metrics = False
        self._refresh_history(preferred_agent_name, preserve_selection=False)
        self._load_selected_agent_metrics()

    def refresh_history(
        self,
        preferred_agent_name: str | None = None,
    ) -> None:
        """Reload the history list without changing a live metrics view."""
        if self._following_live_metrics:
            self._refresh_history(preferred_agent_name, preserve_selection=False)
            return

        self._refresh_history(None, preserve_selection=True)
        if not self._following_live_metrics:
            self._load_selected_agent_metrics()

    def set_terminal_text(self, text: str) -> None:
        """Render a status message in the train/test terminal panel."""
        self.terminal_section.set_text(text)

    def set_training_enabled(self, enabled: bool) -> None:
        """Enable or disable train/test actions."""
        self.buttons_section.set_training_enabled(enabled)

    def flush_ui_updates(self) -> None:
        """Force pending UI state changes to render before long-running work."""
        self.update_idletasks()

    def show_live_metrics(self, metrics: dict[str, list[float]]) -> None:
        """Render live training metrics without switching history selection."""
        self._following_live_metrics = True
        self.metrics_section.set_metrics_data(metrics)

    def _get_selected_metrics_csv_path(self) -> Path | None:
        """Resolve metrics CSV path for the selected agent."""
        if self.project_dir is None or self.selected_agent_name is None:
            return None
        return (
            self.project_dir
            / "agents"
            / self.selected_agent_name
            / "training_metrics.csv"
        )

    def _on_agent_row_click(self, agent_name: str) -> None:
        """Handle selecting an agent row in history."""
        self.selected_agent_name = agent_name
        self._following_live_metrics = False
        self.header_section.set_last_training_session(agent_name)
        self._load_selected_agent_metrics()

    def _on_download_click(self, _agent_name: str) -> None:
        """Placeholder action for downloading an agent."""
        print("download clicked")

    def _on_test_selected_agent_click(self) -> None:
        """Placeholder action for running selected-agent test."""
        print("clicked")

    def persist_ui_state(self) -> None:
        """Persist the current non-training UI state across page recreation."""
        if self.controller is None or not hasattr(
            self.controller.app,
            "set_page_state",
        ):
            return
        self.controller.app.set_page_state(
            "train_test",
            {
                "project_name": self.project_name,
                "selected_agent_name": self.selected_agent_name,
                "selected_metrics": self.metrics_section.get_selected_metrics(),
            },
        )

    def _restore_ui_state(self) -> dict[str, object]:
        if self.controller is None or not hasattr(
            self.controller.app,
            "get_page_state",
        ):
            return {}
        state = self.controller.app.get_page_state("train_test")
        if state.get("project_name") != self.project_name:
            return {}
        return state
