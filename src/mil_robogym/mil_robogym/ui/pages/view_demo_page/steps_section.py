import tkinter as tk
from typing import Callable

from mil_robogym.data_collection.types import Coord4D


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
        super().__init__(
            parent,
            bg="#C0C0C0" if is_origin else "#E6E6E6",
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
            bg="#C0C0C0" if is_origin else "#E6E6E6",
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
        self.config(bg="#E6E6E6")
        self.label.config(bg="#E6E6E6")


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

        self.grid_propagate(False)

        tk.Label(
            self,
            text="Steps",
            bg="#CFCFCF",
            font=("Arial", 12, "bold"),
            anchor="w",
        ).pack(fill="x", padx=8, pady=(8, 4))

        self.canvas = tk.Canvas(
            self,
            bg="#CFCFCF",
            highlightthickness=0,
            width=260,
        )
        self.canvas.pack(side="left", fill="y")

        scrollbar = tk.Scrollbar(self, command=self.canvas.yview)
        scrollbar.pack(side="right", fill="y")

        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.steps_frame = tk.Frame(self.canvas, bg="#CFCFCF")
        self.canvas.create_window((0, 0), window=self.steps_frame, anchor="nw")

        self.steps_frame.bind(
            "<Configure>",
            lambda _: self.canvas.configure(
                scrollregion=self.canvas.bbox("all"),
            ),
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
            lambda _, i=index, c=coordinate: self._select_step(i, c),
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

    def move_highlight(self, from_i: int, to_i: int) -> None:

        self.steps[from_i].default_color()
        self.steps[to_i].highlight()

    def _select_step(self, index: int, coordinate: Coord4D) -> None:

        # Move sub to coordinate
        self.controller.move_model_to(coordinate)

        # Remove border from last step
        if 0 <= self.selected_index < len(self.steps):
            self.steps[self.selected_index].config(highlightthickness=0)

        self.steps[index].config(
            highlightbackground="green",
            highlightcolor="green",
            highlightthickness=3,
        )

        self.selected_index = index
