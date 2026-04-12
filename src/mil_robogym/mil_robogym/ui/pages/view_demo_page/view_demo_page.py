import tkinter as tk
from contextlib import suppress
from typing import Any

from mil_robogym.ui.components.scrollable_frame import ScrollableFrame
from mil_robogym.ui.components.tool_tip import ToolTipGroup

from .collected_data_section import CollectedDataSection
from .controls_section import ControlsSection
from .demo_view_controller import DemoViewController
from .header import Header
from .steps_section import StepsSection

CONTROL_MASK = 0x4
SHIFT_MASK = 0x1
TEXT_INPUT_CLASSES = {"Entry", "TEntry", "Text", "Spinbox", "TSpinbox", "TCombobox"}


class ViewDemoPage(tk.Frame):
    """
    UI Page for viewing a demo from a project.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#DADADA")
        self.controller = DemoViewController(self, controller)
        self._tooltip_group = ToolTipGroup()
        self._root = self.winfo_toplevel()
        self._root_bindings: list[tuple[str, str]] = []
        self._ctrl_tooltips_visible = False

        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Bindings
        self.bind("<space>", self.controller.pause_recording)

        # Header
        self.header = Header(self, self.controller)
        self.header.grid(row=0, column=0, sticky="ew", padx=14, pady=(14, 8))
        self.header.subtitle.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 8))

        # Content
        self.content = ScrollableFrame(self, bg="#DADADA", fill_height=True)
        self.content.grid(row=2, column=0, sticky="nsew", padx=14)
        self.content.content.grid_rowconfigure(0, weight=1)
        self.content.content.grid_columnconfigure(1, weight=1)

        # Steps section
        self.steps = StepsSection(self.content.content, self.controller)
        self.steps.grid(row=0, column=0, sticky="nsw", padx=(0, 10))

        # Data collected section
        self.data_section = CollectedDataSection(self.content.content)

        # Controls
        self.controls = ControlsSection(self, self.controller)
        self.controls.grid(row=3, column=0, sticky="ew", padx=14, pady=(6, 14))

        self._install_tooltips()
        self._bind_shortcuts()
        self.bind("<Destroy>", self._on_destroy, add="+")

    def set_context(self, project: dict[str, Any] | None = None, **kwargs: Any):
        self.content.reset_scroll()
        self.controller.set_context(project, demo=kwargs.get("demo", {}))

    def cleanup(self) -> None:
        self._hide_shortcut_tooltips()
        for sequence, funcid in self._root_bindings:
            with suppress(Exception):
                self._root.unbind(sequence, funcid)
        self._root_bindings.clear()
        self._tooltip_group.clear()

    def _install_tooltips(self) -> None:
        self._tooltip_group.add(
            self.header.new_demo_btn,
            self._tooltip_text("Create Fresh Demo", "Ctrl+N"),
            forced_text="Ctrl+N",
        )
        self._tooltip_group.add(
            self.header.edit_btn,
            self._tooltip_text("Edit Demo Settings", "Ctrl+E"),
            forced_text="Ctrl+E",
        )
        self._tooltip_group.add(
            self.steps.play_sequence_button,
            self._tooltip_text("Preview Recorded Steps", "Ctrl+Shift+P"),
            forced_text="Ctrl+Shift+P",
        )
        self._tooltip_group.add(
            self.controls.play_button,
            self._tooltip_text("Start or Continue Recording", "Ctrl+P"),
            forced_text="Ctrl+P",
        )
        self._tooltip_group.add(
            self.controls.pause_button,
            self._tooltip_text("Pause Recording", "Ctrl+P"),
            forced_text="Ctrl+P",
        )
        self._tooltip_group.add(
            self.controls.undo_button,
            self._tooltip_text("Undo Step", "Ctrl+Z"),
            forced_text="Ctrl+Z",
        )
        self._tooltip_group.add(
            self.controls.redo_button,
            self._tooltip_text("Redo Step", "Ctrl+Y or Ctrl+Shift+Z"),
            forced_text="Ctrl+Y",
        )
        self._tooltip_group.add(
            self.controls.reset_demo_button,
            self._tooltip_text("Reset Demo Data", "Ctrl+Shift+R"),
            forced_text="Ctrl+Shift+R",
        )
        self._tooltip_group.add(
            self.controls.preposition_button,
            self._tooltip_text("Grab Start Position", "Ctrl+M"),
            forced_text="Ctrl+M",
        )
        self._tooltip_group.add(
            self.controls.random_position_button,
            self._tooltip_text("Random Start Position", "Ctrl+Shift+M"),
            forced_text="Ctrl+Shift+M",
        )

    def _bind_shortcuts(self) -> None:
        self._bind_root("<KeyPress>", self._on_root_key_press)
        self._bind_root("<KeyRelease>", self._on_root_key_release)

    def _bind_root(self, sequence: str, handler) -> None:
        funcid = self._root.bind(sequence, handler, add="+")
        self._root_bindings.append((sequence, funcid))

    def _on_root_key_press(self, event: tk.Event) -> str | None:
        if not self._page_is_active():
            return None

        if self._event_has_control(event):
            self._show_shortcut_tooltips()

        if not self._can_handle_shortcuts():
            return None

        shortcut = self._shortcut_from_event(event)
        if shortcut is None:
            return None

        shortcut_actions = {
            ("n", False): self.controller.show_create_demo,
            ("e", False): self.controller.show_edit_demo,
            ("p", False): self.controller.toggle_recording,
            ("p", True): self.controller.toggle_sequence_playback,
            ("z", False): self.controller.undo_step,
            ("z", True): self.controller.redo_step,
            ("y", False): self.controller.redo_step,
            ("m", False): self.controller.preposition,
            ("m", True): self.controller.set_random_origin,
            ("r", True): self.controller.reset_demo,
        }
        action = shortcut_actions.get(shortcut)
        if action is None:
            return None

        self._hide_shortcut_tooltips()
        action()
        return "break"

    def _on_root_key_release(self, event: tk.Event) -> None:
        if not self._page_is_active():
            return

        if event.keysym in {"Control_L", "Control_R"} or not self._event_has_control(
            event,
        ):
            self._hide_shortcut_tooltips()

    def _page_is_active(self) -> bool:
        with suppress(tk.TclError):
            return bool(
                self.winfo_exists()
                and hasattr(self, "controls")
                and self.controls.winfo_exists(),
            )
        return False

    def _can_handle_shortcuts(self) -> bool:
        return (
            not self.controller.has_active_popup() and not self._focus_is_text_input()
        )

    def _focus_is_text_input(self) -> bool:
        focus_widget = self._root.focus_get()
        if focus_widget is None:
            return False
        with suppress(tk.TclError):
            return focus_widget.winfo_class() in TEXT_INPUT_CLASSES
        return False

    def _shortcut_from_event(self, event: tk.Event) -> tuple[str, bool] | None:
        if not self._event_has_control(event):
            return None

        if event.keysym in {"Control_L", "Control_R", "Shift_L", "Shift_R"}:
            return None

        key = event.keysym.lower()
        if len(key) != 1 or not key.isalpha():
            return None

        return key, self._event_has_shift(event)

    def _event_has_control(self, event: tk.Event) -> bool:
        return bool(event.state & CONTROL_MASK) or event.keysym in {
            "Control_L",
            "Control_R",
        }

    def _event_has_shift(self, event: tk.Event) -> bool:
        return bool(event.state & SHIFT_MASK) or event.keysym.isupper()

    def _show_shortcut_tooltips(self) -> None:
        if self._ctrl_tooltips_visible or self.controller.has_active_popup():
            return
        self._tooltip_group.show_all()
        self._ctrl_tooltips_visible = True

    def _hide_shortcut_tooltips(self) -> None:
        if not self._ctrl_tooltips_visible:
            return
        self._tooltip_group.hide_all()
        self._ctrl_tooltips_visible = False

    def _on_destroy(self, event: tk.Event) -> None:
        if event.widget is self:
            self.cleanup()

    @staticmethod
    def _tooltip_text(label: str, shortcut: str) -> str:
        return f"{label}\n{shortcut}"
