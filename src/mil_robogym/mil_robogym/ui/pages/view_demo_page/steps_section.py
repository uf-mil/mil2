import tkinter as tk


class StepsSection(tk.Frame):
    """
    Left panel displaying demo steps in a scrollable region.
    """

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, bg="#CFCFCF", width=260)

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

    def add_step(self, text: str, highlight: bool = False) -> None:
        color = "#E6E6E6" if not highlight else "#CFE8FF"

        row = tk.Frame(self.steps_frame, bg=color, bd=1, relief="solid")
        row.pack(fill="x", padx=6, pady=4)

        tk.Label(row, text=text, bg=color, font=("Courier", 10)).pack(
            side="left",
            padx=6,
            pady=6,
        )

    def clear(self) -> None:
        for child in self.steps_frame.winfo_children():
            child.destroy()
