import tkinter as tk

from mil_robogym.ui.pages.create_project_page.create_project_page import (
    CreateProjectPage,
)
from mil_robogym.ui.pages.project_page.project_page import ProjectPage
from mil_robogym.ui.pages.start_page.start_page import StartPage


class RoboGymApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("MIL RoboGYM")
        self.configure(bg="#DADADA")
        self.geometry("560x360")
        self.minsize(460, 300)

        self.container = tk.Frame(self, bg="#DADADA")
        self.container.pack(fill="both", expand=True)
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.pages = {}
        self._register_page("start", StartPage)
        self._register_page("create_project", CreateProjectPage)
        self._register_page("project", ProjectPage)
        self.show_page("start")

    def _register_page(self, name, page_cls):
        frame = page_cls(self.container, controller=self)
        self.pages[name] = frame
        frame.grid(row=0, column=0, sticky="nsew")

    def show_page(self, name, **kwargs):
        page = self.pages.get(name)
        if page is not None:
            if kwargs and hasattr(page, "set_context"):
                page.set_context(**kwargs)
            page.tkraise()


def main():
    app = RoboGymApp()
    app.mainloop()
