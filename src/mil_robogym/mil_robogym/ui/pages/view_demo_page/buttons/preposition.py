import tkinter as tk

from mil_robogym.clients.get_pose_client import GetPoseClient
from mil_robogym.clients.world_control_client import WorldControlClient
from mil_robogym.data_collection.filesystem import edit_demo
from mil_robogym.ui.components.grab_coordinates_popup import GrabCoordinatesPopup
from mil_robogym.ui.components.keyboard_controls_gui import KeyboardControlsGUI


class PrepositionButton:
    """
    Button that allows you to align the sub to the desired starting position.
    """

    def __init__(
        self,
        grand_parent: tk.Widget,
        parent: tk.Frame,
        isEnabled: bool = False,
    ) -> None:

        self.grand_parent = grand_parent

        self.isEnabled = isEnabled

        self.get_pose_client = GetPoseClient()
        self.world_control_client = WorldControlClient()

        self.keyboard_controls_gui = None
        self.popup = None

        tk.Button(
            parent,
            text="Preposition",
            state="active" if isEnabled else "disabled",
            width=12,
            command=self._on_click,
        ).pack(side="left")

    def _on_click(self) -> None:
        if self.isEnabled:

            # Play simulation
            self.world_control_client.play_simulation()

            # Start up keyboard controls
            self.keyboard_controls_gui = (
                self.keyboard_controls_gui
                or KeyboardControlsGUI(
                    self.grand_parent,
                    self._on_close_of_keyboard_controls,
                )
            )
            self.keyboard_controls_gui.show()

            # Create a pop up to record coordinate
            if self.popup and self.popup.win.winfo_exists():
                self.popup.win.lift()
                self.popup.win.focus_force()
                self.keyboard_controls_gui.show()
                return

            self.popup = GrabCoordinatesPopup(
                self.grand_parent,
                self.get_pose_client.send_request,
                self._save_coordinate,
                num_coordinates=1,
            )

    def _on_close_of_keyboard_controls(self):
        """
        Close "Grabbing Coordinates From Simulation" popup.
        """
        self.popup.finish()

    def _save_coordinate(self, coordinate):
        """
        Saves coordinate to demo's config.yaml and displays new origin on the GUI.
        """
        c1 = coordinate[0]

        if c1:
            x, y, z, yaw = c1

            project = self.grand_parent.project["robogym_project"]
            demo = self.grand_parent.demo
            demo["start_position"] = (x, y, z, yaw)

            # Update config
            edit_demo(project, demo)

            # Display changes in UI
            self.grand_parent.steps.clear()
            self.grand_parent.steps.add_step(
                f"Origin: ({x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f})",
            )

        # Close out windows
        self.keyboard_controls_gui.hide()
        self.world_control_client.pause_simulation()
        self.popup = None
