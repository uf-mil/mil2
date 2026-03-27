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
        super().__init__(parent, bg="#E6E6E6", bd=1, relief="solid")
        self.pack(fill="x", padx=6, pady=4)

        self.coordinate = coordinate

        x, y, z, yaw = coordinate
        text = (
            "Origin: " if is_origin else ""
        ) + f"({x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f})"

        label = tk.Label(self, text=text, bg="#E6E6E6", font=("Courier", 8))
        label.pack(
            side="left",
            padx=6,
            pady=6,
        )
        label.bind("<Button-1>", on_click)


class StepsSection(tk.Frame):
    """
    Left panel displaying demo steps in a scrollable region.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#CFCFCF", width=260)
        self.controller = controller

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
        self.controller.last_pose = coordinate
        Step(
            self.steps_frame,
            lambda _: self.controller.move_model_to(coordinate),
            coordinate,
            is_origin,
        )

    def clear(self) -> None:
        for child in self.steps_frame.winfo_children():
            child.destroy()
