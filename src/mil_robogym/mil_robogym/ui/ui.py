import tkinter as tk
from typing import Any, Type

from mil_robogym.ui.pages.create_project_page.create_project_page import (
    CreateProjectPage,
)
from mil_robogym.ui.pages.project_page.project_page import ProjectPage
from mil_robogym.ui.pages.start_page.start_page import StartPage


class RoboGymApp(tk.Tk):
    """
    Main Tkinter application class for the MIL RoboGYM UI.
    This class is responsible for initializing the root window,
    registering page frames, and handling page navigation.
    """

    def __init__(self) -> None:
        """Initialize the RoboGym application window and page container."""
        super().__init__()

        self.title("MIL RoboGYM")
        self.configure(bg="#DADADA")

        self.geometry("950x540")
        self.minsize(950, 540)

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.container: tk.Frame = tk.Frame(self, bg="#DADADA")
        self.container.grid(row=0, column=0, sticky="nsew")
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.pages: dict[str, tk.Frame] = {}
        self._register_page("start", StartPage)
        self._register_page("create_project", CreateProjectPage)
        self._register_page("project", ProjectPage)

        self.show_page("start")

    def _register_page(self, name: str, page_cls: Type[tk.Frame]) -> None:
        """
        Instantiate and register a page frame with the application.
        The page is created, stored by name, and gridded into the shared container.

        :param name: Unique name used to reference the page.
        :type name: str
        :param page_cls: Frame class implementing the page UI.
        :type page_cls: type[tk.Frame]
        """
        frame = page_cls(self.container, controller=self)
        self.pages[name] = frame
        frame.grid(row=0, column=0, sticky="nsew")

    def show_page(self, name: str, **kwargs: Any) -> None:
        """
        Display the page associated with the given name.

        If the page implements a 'set_context' method, it will be
        called with the provided keyword arguments before the page
        is raised.

        :param name: Name of the page to display.
        :type name: str
        :param kwargs: Optional context data passed to the page.
        """
        page = self.pages.get(name)
        if page is not None:
            if kwargs and hasattr(page, "set_context"):
                page.set_context(**kwargs)
            page.tkraise()


def main() -> None:
    """Application entry point."""
    app = RoboGymApp()
    app.mainloop()
