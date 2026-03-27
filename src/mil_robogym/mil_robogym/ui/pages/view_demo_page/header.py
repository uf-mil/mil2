import tkinter as tk
from typing import Callable


class Header(tk.Frame):
    """
    Top header section showing breadcrumb navigation and demo info.
    """

    def __init__(
        self,
        parent: tk.Widget,
        on_home_click: Callable[[], None],
        on_project_click: Callable[[], None],
    ) -> None:
        super().__init__(parent, bg="#DADADA")

        self._on_home_click = on_home_click
        self._on_project_click = on_project_click

        self.grid_columnconfigure(0, weight=1)

        self._title_left = tk.Frame(self, bg="#DADADA")
        self._title_left.grid(row=0, column=0, sticky="w")

        self.home_title = tk.Label(
            self._title_left,
            text="MIL RoboGYM >",
            bg="#DADADA",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.home_title.pack(side="left")
        self.home_title.bind("<Button-1>", lambda _: self._on_home_click())

        self.project_title = tk.Label(
            self._title_left,
            text="Project >",
            bg="#DADADA",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.project_title.pack(side="left", padx=(6, 0))
        self.project_title.bind("<Button-1>", lambda _: self._on_project_click())

        self.demo_title = tk.Label(
            self._title_left,
            text="Demo",
            bg="#DADADA",
            font=("Arial", 20, "bold"),
        )
        self.demo_title.pack(side="left", padx=(6, 0))

        self.edit_btn = tk.Button(self, text="Edit", width=8)
        self.edit_btn.grid(row=0, column=1, sticky="e")

        self.subtitle = tk.Label(
            parent,
            text="",
            bg="#DADADA",
            fg="#333333",
            font=("Arial", 11),
            anchor="w",
        )

    def set_titles(self, project_name: str, demo_name: str) -> None:
        self.project_title.configure(text=f"{project_name} >")
        self.demo_title.configure(text=demo_name)

    def set_subtitle(self, text: str) -> None:
        self.subtitle.configure(text=text)
