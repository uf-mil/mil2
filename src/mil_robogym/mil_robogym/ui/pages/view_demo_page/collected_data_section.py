import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class CollectedDataSection(tk.Frame):
    """
    Main data display area showing collected demo data.
    """

    def __init__(self, parent: tk.Widget, controller) -> None:
        super().__init__(parent, bg="#CFCFCF")

        self.controller = controller

        self.selected_column = tk.StringVar()

        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

        header = tk.Frame(self, bg="#CFCFCF")
        header.grid(row=0, column=0, sticky="ew")

        tk.Label(
            header,
            text="Collected Data",
            bg="#CFCFCF",
            font=("Arial", 12, "bold"),
        ).pack(side="left", padx=8, pady=6)

        # Dropdown
        self.dropdown = ttk.Combobox(
            header,
            textvariable=self.selected_column,
            state="readonly",
        )
        self.dropdown.pack(fill="x", expand=True, side="right", padx=8)
        self.dropdown.bind("<<ComboboxSelected>>", self.controller.update_graph)

        self.data_display = tk.Frame(
            self,
            bg="#EAEAEA",
            bd=1,
            relief="sunken",
        )
        self.data_display.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))

        # Matplotlib figure
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.data_display)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
