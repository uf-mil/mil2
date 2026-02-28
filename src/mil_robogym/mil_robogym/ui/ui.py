import tkinter as tk
from typing import Any, Type

from mil_robogym.ui.pages.create_project_page.create_project_page import (
    CreateProjectPage,
)
from mil_robogym.ui.pages.edit_project_page.edit_project_page import EditProjectPage
from mil_robogym.ui.pages.start_page.start_page import StartPage
from mil_robogym.ui.pages.view_project_page.view_project_page import ViewProjectPage


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

        self.page_classes: dict[str, Type[tk.Frame]] = {}
        self.pages: dict[str, tk.Frame] = {}
        self._register_page("start", StartPage)
        self._register_page("create_project", CreateProjectPage)
        self._register_page("edit_project", EditProjectPage)
        self._register_page("view_project", ViewProjectPage)

        self.show_page("start")

    def _register_page(self, name: str, page_cls: Type[tk.Frame]) -> None:
        """
        Register a page class by name.

        :param name: Unique name used to reference the page.
        :type name: str
        :param page_cls: Frame class implementing the page UI.
        :type page_cls: type[tk.Frame]
        """
        self.page_classes[name] = page_cls

    def _get_or_create_page(self, name: str) -> tk.Frame | None:
        """
        Return an existing page instance or create it on first access.

        :param name: Registered page name.
        :type name: str
        :return: Existing or newly created page frame, or 'None' if unknown.
        :rtype: tk.Frame | None
        """
        page = self.pages.get(name)
        if page is not None:
            return page

        page_cls = self.page_classes.get(name)
        if page_cls is None:
            return None

        page = page_cls(self.container, controller=self)
        self.pages[name] = page
        page.grid(row=0, column=0, sticky="nsew")
        return page

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
        page = self._get_or_create_page(name)
        if page is not None:
            if hasattr(page, "set_context"):
                page.set_context(**kwargs)
            page.tkraise()


def main() -> None:
    """Application entry point."""
    app = RoboGymApp()
    app.mainloop()
