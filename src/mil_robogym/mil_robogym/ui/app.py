import tkinter as tk
from typing import Any, Type

from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.ui.pages.create_project_page.create_project_page import (
    CreateProjectPage,
)
from mil_robogym.ui.pages.edit_project_page.edit_project_page import EditProjectPage
from mil_robogym.ui.pages.start_page.start_page import StartPage
from mil_robogym.ui.pages.train_test_page.train_test_page import TrainTestPage
from mil_robogym.ui.pages.training_settings_page.training_settings_page import (
    TrainingSettingsPage,
)
from mil_robogym.ui.pages.view_demo_page.view_demo_page import ViewDemoPage
from mil_robogym.ui.pages.view_project_page.view_project_page import ViewProjectPage


class App(tk.Tk):
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
        self.page_state: dict[str, dict[str, Any]] = {}
        self._register_page("start", StartPage)
        self._register_page("create_project", CreateProjectPage)
        self._register_page("edit_project", EditProjectPage)
        self._register_page("view_project", ViewProjectPage)
        self._register_page("train_test", TrainTestPage)
        self._register_page("training_settings", TrainingSettingsPage)
        self._register_page("view_demo", ViewDemoPage)

        self.show_page("start")

        # Start and stop simulation to get all topics
        world_control_client = WorldControlClient()
        world_control_client.play_simulation()
        world_control_client.pause_simulation()

    def _register_page(self, name: str, page_cls: Type[tk.Frame]) -> None:
        """
        Instantiate and register a page frame with the application.
        The page is created, stored by name, and gridded into the shared container.

        :param name: Unique name used to reference the page.
        :type name: str
        :param page_cls: Frame class implementing the page UI.
        :type page_cls: type[tk.Frame]
        """
        self.pages[name] = page_cls

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
        page_cls = self.pages.get(name)

        if page_cls is not None:

            frame = page_cls(self.container, controller=self)
            frame.grid(row=0, column=0, sticky="nsew")

            if hasattr(frame, "set_context"):
                frame.set_context(**kwargs)

            frame.tkraise()

    def set_page_state(self, name: str, state: dict[str, Any]) -> None:
        """Persist small UI state across page recreation."""
        self.page_state[name] = dict(state)

    def get_page_state(self, name: str) -> dict[str, Any]:
        """Return previously persisted UI state for a page."""
        return dict(self.page_state.get(name, {}))
