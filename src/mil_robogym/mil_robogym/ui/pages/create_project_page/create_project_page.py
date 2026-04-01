from __future__ import annotations

import tkinter as tk
from typing import Any

from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.filesystem import create_project_folder
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.data_collection.get_ros2_topics import get_ros2_topics
from mil_robogym.data_collection.types import Coord4D, RoboGymProjectYaml
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI

from .controls_section import ControlsSection
from .header_section import HeaderSection
from .project_fields_section import ProjectFieldsSection
from .random_spawn_section import RandomSpawnSection
from .sub_topics_section import SubTopicsSection
from .tensor_spec_section import TensorSpecSection
from .topics_section import TopicsSection


class CreateProjectPage(tk.Frame):
    """Render and coordinate the Create Project workflow screen. This keeps behavior scoped to the current component.

    Args:
        parent: Parent Tk widget that owns this page frame.
        controller: Optional app controller used for page navigation.
    Returns:
        None.
    """

    def __init__(
        self,
        parent: tk.Widget,
        controller,
    ) -> None:
        """Initialize page services, sections, and form-state wiring. This keeps behavior scoped to the current component.

        Args:
            parent: Parent widget where this page is rendered.
            controller: Optional navigation controller implementing show_page.
        Returns:
            None.
        """
        super().__init__(parent, bg="#DADADA")

        self.controller = controller
        self.world_control_client = WorldControlClient()
        self.gz_pose_client = GetPoseClient()

        self.keyboard_controls_gui: KeyboardControlsGUI | None = None
        self.popup: GrabCoordinatesPopup | None = None

        self._topics = self._safe_get_topics()
        self._world_default = self._safe_get_world_file()

        self.selected_input_topics: list[str] = []
        self.selected_output_topics: list[str] = []
        self.selected_input_topic_subtopics: dict[str, list[str]] = {}
        self.selected_output_topic_subtopics: dict[str, list[str]] = {}

        self.header_section = HeaderSection(self, self._on_home_title_click)
        self.project_fields_section = ProjectFieldsSection(
            self,
            self._world_default,
            self._on_form_state_changed,
        )

        self.controls_section = ControlsSection(
            self,
            self._on_cancel,
            self._on_create_project,
        )

        self.random_spawn_section = RandomSpawnSection(
            self,
            self._on_form_state_changed,
            self._on_grab_from_sim,
        )

        self.topics_section = TopicsSection(
            self,
            self._topics,
            self._on_topics_selection_changed,
        )

        self.sub_topics_section = SubTopicsSection(
            self,
            self._on_subtopics_selection_changed,
        )

        self.tensor_spec_section = TensorSpecSection(
            self,
            lambda: self._build_project_config(ensure_subtopics_loaded=True),
            self._get_selected_input_topics,
            self._get_selected_output_topics,
            self._get_selected_input_topic_subtopics,
            self._get_selected_output_topic_subtopics,
            self.controls_section.show_error_tooltip,
            self.controls_section.hide_error_tooltip,
        )

        self._sync_subtopics_with_topics()

        for col in range(6):
            self.grid_columnconfigure(col, weight=1, uniform="half")
        self.grid_rowconfigure(7, weight=1)
        self.grid_rowconfigure(9, weight=1)

        self._update_create_project_button_state()

    def _safe_get_topics(self) -> list[str]:
        """Fetch ROS topics and normalize failures into a page-specific error. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            List of discovered ROS topic names.
        """
        try:
            return get_ros2_topics()
        except (RuntimeError, FileNotFoundError) as exc:
            raise RuntimeError(
                "Getting ROS 2 topics on create projects page failed",
            ) from exc

    def _safe_get_world_file(self) -> str:
        """
        Return the default world-file path used for new projects. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            World-file path string used to initialize the disabled field.
        """
        return "~/mil2/install/subjugator_gazebo/share/subjugator_gazebo/worlds/robosub_2025.world"

    def _on_form_state_changed(self) -> None:
        """Handle section-level state changes by refreshing form validity state. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self._update_create_project_button_state()

    def _on_grab_from_sim(self) -> None:
        """Start simulation controls and open the coordinate-grab popup. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
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
            self._on_popup_finished,
        )

    def _on_popup_finished(self, coords: list[Coord4D | None]) -> None:
        """Handle popup completion by normalizing returned coordinate list.

        Args:
            coords: Coordinate list returned by the popup finish callback.
        Returns:
            None.
        """
        c1 = coords[0] if len(coords) > 0 else None
        c2 = coords[1] if len(coords) > 1 else None
        self._display_collected_coords(c1, c2)

    def _display_collected_coords(
        self,
        coords: list[Coord4D],
    ) -> None:
        """Apply collected simulation coordinates and pause simulation state. This keeps behavior scoped to the current component.

        Args:
            c1: First collected coordinate, or None when unavailable.
            c2: Second collected coordinate, or None when unavailable.
        Returns:
            None.
        """
        c1, c2 = coords
        self.random_spawn_section.set_collected_coords(c1, c2)

        if self.keyboard_controls_gui is not None:
            self.keyboard_controls_gui.hide()
        self.world_control_client.pause_simulation()
        self.popup = None
        self._update_create_project_button_state()

    def _on_close_of_keyboard_controls(self) -> None:
        """Handle close events from the keyboard-controls popup. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        if self.popup is not None:
            self.popup.finish()

    def _on_cancel(self) -> None:
        """Handle cancel actions by navigating back to the start page. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        if self.controller is not None:
            self.controller.show_page("start")

    def _on_home_title_click(self, _event: object) -> None:
        """Handle breadcrumb clicks by navigating to the start page. This keeps behavior scoped to the current component.

        Args:
            _event: Tk click event object passed by the bound label.
        Returns:
            None.
        """
        if self.controller is not None:
            self.controller.show_page("start")

    def _build_project_config(
        self,
        *,
        ensure_subtopics_loaded: bool = False,
    ) -> RoboGymProjectYaml:
        """Build the project config payload from current section state. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            RoboGymProjectYaml dictionary used for tensor-spec and folder creation.
        """
        coord1, coord2 = self.random_spawn_section.get_coords()
        random_spawn_enabled = self.random_spawn_section.is_random_spawn_enabled()

        return {
            "name": self.project_fields_section.project_name_var.get().strip(),
            "world_file": self.project_fields_section.world_file_var.get().strip(),
            "model_name": self.project_fields_section.model_name_var.get().strip(),
            "random_spawn_space": {
                "enabled": random_spawn_enabled,
                "coord1_4d": [float(v) for v in (coord1 or (0.0, 0.0, 0.0, 0.0))],
                "coord2_4d": [float(v) for v in (coord2 or (0.0, 0.0, 0.0, 0.0))],
            },
            "input_topics": self._selected_input_topic_subtopics(
                ensure_loaded=ensure_subtopics_loaded,
            ),
            "output_topics": self._selected_output_topic_subtopics(
                ensure_loaded=ensure_subtopics_loaded,
            ),
        }

    def _on_create_project(self) -> None:
        """Create the project directory from validated form state. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        if not self._is_form_valid():
            self._update_create_project_button_state()
            return

        self._update_create_project_button_state()
        tensor_spec = self.tensor_spec_section.compute_tensor_spec()
        if tensor_spec is None:
            return

        project_cfg = self._build_project_config(ensure_subtopics_loaded=True)
        project_name = project_cfg["name"]
        project_cfg["tensor_spec"] = tensor_spec

        try:
            create_project_folder(project_cfg)
        except (FileExistsError, OSError, RuntimeError, ValueError, KeyError) as exc:
            self.controls_section.set_create_project_enabled(False)
            self.controls_section.show_error_tooltip(str(exc) or type(exc).__name__)
            return

        if self.controller is None:
            return

        project_payload: dict[str, Any] | None = None
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
        """Validate project-name safety against disallowed filesystem characters. This keeps behavior scoped to the current component.

        Args:
            name: Candidate project name entered by the user.
        Returns:
            True when the name is non-empty and free of forbidden characters.
        """
        if not name.strip():
            return False

        forbidden = set('/\\:*?"<>|')
        return not any(char in forbidden for char in name)

    def _is_form_valid(self) -> bool:
        """Evaluate whether all form constraints are currently satisfied. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            True when name, topic selections, and optional coords are valid.
        """
        project_name = self.project_fields_section.project_name_var.get().strip()
        has_valid_name = self._is_filesystem_safe_name(project_name)
        has_valid_topics = self.topics_section.has_valid_topic_selection()
        self._sync_selected_topic_subtopics_from_section()
        has_valid_subtopics = self._has_valid_subtopic_selection()

        random_spawn_enabled = self.random_spawn_section.is_random_spawn_enabled()
        coord1, coord2 = self.random_spawn_section.get_coords()
        has_valid_coords = (not random_spawn_enabled) or (
            coord1 is not None and coord2 is not None
        )

        return (
            has_valid_name
            and has_valid_topics
            and has_valid_subtopics
            and has_valid_coords
        )

    def _has_valid_subtopic_selection(self) -> bool:
        """Return True when each selected topic has at least one selected subtopic."""
        for topic in self.selected_input_topics:
            if not self.selected_input_topic_subtopics.get(topic):
                return False

        for topic in self.selected_output_topics:
            if not self.selected_output_topic_subtopics.get(topic):
                return False

        return True

    def _update_create_project_button_state(self) -> None:
        """Update Create Project button state from current form validity. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self.controls_section.update_create_project_button_state(self._is_form_valid())

    def _sync_subtopics_with_topics(self) -> None:
        """Sync subtopic panels with currently selected input/output topics."""
        self.selected_input_topics = self.topics_section.get_selected_input_topics()
        self.selected_output_topics = self.topics_section.get_selected_output_topics()

        self.sub_topics_section.set_selected_topics(
            input_topics=self.selected_input_topics,
            output_topics=self.selected_output_topics,
        )

        self._sync_selected_topic_subtopics_from_section()

    def _on_topics_selection_changed(self) -> None:
        """Handle topic list selection changes from TopicsSection."""
        self._sync_subtopics_with_topics()
        self.tensor_spec_section.invalidate_tensor_spec()
        self._on_form_state_changed()

    def _on_subtopics_selection_changed(self) -> None:
        """Handle subtopic selection changes from SubTopicsSection."""
        self._sync_selected_topic_subtopics_from_section()
        self.tensor_spec_section.invalidate_tensor_spec()
        self._on_form_state_changed()

    def _sync_selected_topic_subtopics_from_section(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> None:
        """Update page-level selected subtopic mappings from the subtopics section."""
        self.selected_input_topic_subtopics = (
            self.sub_topics_section.get_selected_input_topic_subtopics(
                ensure_loaded=ensure_loaded,
            )
        )
        self.selected_output_topic_subtopics = (
            self.sub_topics_section.get_selected_output_topic_subtopics(
                ensure_loaded=ensure_loaded,
            )
        )

    def _selected_input_topic_subtopics(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[str]]:
        self._sync_selected_topic_subtopics_from_section(ensure_loaded=ensure_loaded)
        return self.selected_input_topic_subtopics

    def _selected_output_topic_subtopics(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[str]]:
        self._sync_selected_topic_subtopics_from_section(ensure_loaded=ensure_loaded)
        return self.selected_output_topic_subtopics

    def _get_selected_input_topics(self) -> list[str]:
        return list(self.selected_input_topics)

    def _get_selected_output_topics(self) -> list[str]:
        return list(self.selected_output_topics)

    def _get_selected_input_topic_subtopics(self) -> dict[str, list[str]]:
        return {
            topic: list(fields)
            for topic, fields in self.selected_input_topic_subtopics.items()
        }

    def _get_selected_output_topic_subtopics(self) -> dict[str, list[str]]:
        return {
            topic: list(fields)
            for topic, fields in self.selected_output_topic_subtopics.items()
        }
