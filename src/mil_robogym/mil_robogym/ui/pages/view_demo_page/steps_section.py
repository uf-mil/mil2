import tkinter as tk
from typing import Callable

from mil_robogym.data_collection.types import Coord4D
from mil_robogym.ui.components.mouse_wheel import (
    get_mouse_wheel_router,
    scroll_canvas,
)


class Step(tk.Frame):
    """
    UI object for a step.
    """

    def __init__(
        self,
        parent: tk.Frame,
        on_click: Callable,
        coordinate: Coord4D,
        is_origin: bool,
    ):
        self._base_bg = "#C0C0C0" if is_origin else "#E6E6E6"
        super().__init__(
            parent,
            bg=self._base_bg,
            bd=1,
            relief="solid",
            highlightthickness=0,
        )
        self.pack(fill="x", padx=6, pady=4)

        self.coordinate = coordinate

        x, y, z, yaw = coordinate
        text = (
            "Origin: " if is_origin else ""
        ) + f"({x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f})"

        self.label = tk.Label(
            self,
            text=text,
            bg=self._base_bg,
            font=("Courier", 8),
        )
        self.label.pack(
            side="left",
            padx=6,
            pady=6,
        )

        # Bind Click action to all elements
        self.bind("<Button-1>", on_click)
        self.label.bind("<Button-1>", on_click)

    def show(self):
        self.pack(fill="x", padx=6, pady=4)

    def hide(self):
        self.pack_forget()

    def highlight(self):
        self.config(bg="#C6E1F6")
        self.label.config(bg="#C6E1F6")

    def default_color(self):
        self.config(bg=self._base_bg)
        self.label.config(bg=self._base_bg)


class StepsSection(tk.Frame):
    """
    Left panel displaying demo steps in a scrollable region.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#CFCFCF", width=260)
        self.controller = controller

        self.steps = []
        self.current_pose_index = -1
        self.selected_index = -1
        self.last_pose = None
        self._scrollbar_visible = False

        self.grid_propagate(False)

        self.header_row = tk.Frame(self, bg="#CFCFCF")
        self.header_row.pack(fill="x", padx=8, pady=(8, 4))
        self.header_row.grid_columnconfigure(1, weight=1)

        self.title_label = tk.Label(
            self.header_row,
            text="Steps",
            bg="#CFCFCF",
            font=("Arial", 12, "bold"),
            anchor="w",
        )
        self.title_label.grid(row=0, column=0, sticky="w")

        self.play_sequence_button = tk.Button(
            self.header_row,
            text="▶",
            command=self.controller.toggle_sequence_playback,
            state=tk.DISABLED,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 9, "bold"),
            padx=6,
            pady=0,
            cursor="hand2",
        )
        self.play_sequence_button.grid(row=0, column=2, sticky="e")

        self.countdown_label = tk.Label(
            self.header_row,
            text="",
            bg="#CFCFCF",
            fg="#4A4A4A",
            font=("Arial", 9),
            anchor="e",
        )
        self.countdown_label.grid(row=0, column=1, sticky="e", padx=(12, 8))

        self.status_label = tk.Label(
            self,
            text="",
            bg="#CFCFCF",
            fg="#4A4A4A",
            font=("Arial", 9),
            anchor="w",
        )
        self.status_label.pack(fill="x", padx=8, pady=(0, 6))

        self.canvas = tk.Canvas(
            self,
            bg="#CFCFCF",
            highlightthickness=0,
            width=260,
        )
        self.canvas.pack(side="left", fill="both", expand=True)

        self.scrollbar = tk.Scrollbar(self, command=self.canvas.yview)
        self._show_scrollbar()

        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.steps_frame = tk.Frame(self.canvas, bg="#CFCFCF")
        self._steps_window = self.canvas.create_window(
            (0, 0),
            window=self.steps_frame,
            anchor="nw",
        )

        self.steps_frame.bind("<Configure>", self._on_steps_frame_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self._mouse_wheel_router = get_mouse_wheel_router(self)
        self._mouse_wheel_binding = self._mouse_wheel_router.register(
            self,
            self._handle_mouse_wheel,
        )

    def add_step(self, coordinate: Coord4D, is_origin: bool = False) -> None:

        self.current_pose_index += 1
        index = self.current_pose_index
        self.controller.last_pose = coordinate

        # Remove highlight from last non-origin step
        if len(self.steps) > 1:
            self.steps[-1].default_color()

        step = Step(
            self.steps_frame,
            lambda _, i=index: self.controller.select_step(i),
            coordinate,
            is_origin,
        )

        if not is_origin:
            step.highlight()

        self.steps.append(step)

    def clear(self) -> None:

        for step in self.steps:
            step.destroy()

        self.steps = []
        self.current_pose_index = -1
        self.selected_index = -1
        self.set_status_message("")
        self.set_countdown_message("")
        self.set_play_sequence_button_state(enabled=False, is_playing=False)
        self.after_idle(self._update_scrollbar_state)

    def set_status_message(self, message: str) -> None:
        self.status_label.config(text=message)

    def set_countdown_message(self, message: str) -> None:
        self.countdown_label.config(text=message)

    def set_play_sequence_button_state(
        self,
        *,
        enabled: bool,
        is_playing: bool,
    ) -> None:
        self.play_sequence_button.config(
            text="■" if is_playing else "▶",
            state=tk.NORMAL if enabled else tk.DISABLED,
        )

    def refresh_display(self) -> None:

        for i, step in enumerate(self.steps):
            if i <= self.current_pose_index:
                step.show()
            else:
                step.hide()

    def destroy_undone_steps(self) -> None:

        for i in range(self.current_pose_index + 1, len(self.steps)):
            self.steps[i].destroy()

        self.steps = self.steps[: (self.current_pose_index + 1)]

    def get_current_pose(self) -> Coord4D | None:

        if self.steps:
            return self.steps[self.current_pose_index].coordinate

    def get_step_coordinate(self, index: int) -> Coord4D | None:

        if 0 <= index < len(self.steps):
            return self.steps[index].coordinate

        return None

    def move_highlight(self, from_i: int, to_i: int) -> None:

        self.steps[from_i].default_color()
        self.steps[to_i].highlight()

    def set_selected_step(self, index: int | None) -> None:

        if 0 <= self.selected_index < len(self.steps):
            self.steps[self.selected_index].config(highlightthickness=0)

        if index is None or not (0 <= index < len(self.steps)):
            self.selected_index = -1
            return

        self.steps[index].config(
            highlightbackground="green",
            highlightcolor="green",
            highlightthickness=3,
        )

        self.selected_index = index

    def clear_selected_step(self) -> None:
        self.set_selected_step(None)

    def _on_steps_frame_configure(self, _event: tk.Event | None = None) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self._update_scrollbar_state()

    def _on_canvas_configure(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self._steps_window, width=event.width)
        self._update_scrollbar_state()

    def _update_scrollbar_state(self) -> None:
        content_height = self.steps_frame.winfo_reqheight()
        viewport_height = self.canvas.winfo_height()
        if viewport_height <= 1:
            return

        if content_height <= viewport_height:
            self.canvas.yview_moveto(0.0)
            self._hide_scrollbar()
            return

        self._show_scrollbar()

    def _show_scrollbar(self) -> None:
        if self._scrollbar_visible:
            return
        self.scrollbar.pack(side="right", fill="y")
        self._scrollbar_visible = True

    def _hide_scrollbar(self) -> None:
        if not self._scrollbar_visible:
            return
        self.scrollbar.pack_forget()
        self._scrollbar_visible = False

    def _handle_mouse_wheel(self, units: int) -> bool:
        return scroll_canvas(self.canvas, units)
