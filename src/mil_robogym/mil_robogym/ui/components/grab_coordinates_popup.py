import tkinter as tk


class GrabCoordinatesPopup:
    def __init__(self, parent, record_callback, finish_callback):
        """
        parent            : root Tk window
        record_callback() : function that calls your ROS2 service
                            and RETURNS a 4D coordinate (tuple/list)
        """
        self.parent = parent
        self.record_callback = record_callback
        self.finish_callback = finish_callback

        self.coords = [None, None]
        self.index = 0  # which coordinate we are recording

        # Build window
        self.win = tk.Toplevel(parent)
        self.win.title("Grabbing Coordinates From Simulation")
        self.win.geometry("420x180")

        # Make it modal
        self.win.transient(parent)
        self.win.attributes("-topmost", True)

        container = tk.Frame(self.win, padx=20, pady=20)
        container.pack(fill="both", expand=True)

        self.label1 = tk.Label(
            container,
            text="4D-Coordinate 1: Not Recorded",
            anchor="w",
            font=("Arial", 11),
        )
        self.label1.pack(fill="x", pady=5)

        self.label2 = tk.Label(
            container,
            text="4D-Coordinate 2: Not Recorded",
            anchor="w",
            font=("Arial", 11),
        )
        self.label2.pack(fill="x", pady=5)

        self.record_btn = tk.Button(
            container,
            text="Record Coordinate",
            command=self.record_coordinate,
            height=2,
            width=20,
        )
        self.record_btn.pack(pady=15)

        self.win.protocol("WM_DELETE_WINDOW", self.finish)

    def record_coordinate(self):
        """Called when button is pressed."""

        coord = self.record_callback()  # GetGZPose Response
        if coord is None:
            return

        self.coords[self.index] = (coord.x, coord.y, coord.z, coord.yaw)

        formatted = f"({coord.x:.2f}, {coord.y:.2f}, {coord.z:.2f}, {coord.yaw:.2f})"

        if self.index == 0:
            self.label1.config(text=f"4D-Coordinate 1: {formatted}")
        else:
            self.label2.config(text=f"4D-Coordinate 2: {formatted}")

        self.index += 1

        # If we captured both → auto close
        if self.index >= 2:
            self.finish()

    def finish(self):
        """
        Process coordinates through finish callback.
        """
        c1, c2 = self.coords
        self.win.destroy()

        if self.finish_callback:
            self.finish_callback(c1, c2)
