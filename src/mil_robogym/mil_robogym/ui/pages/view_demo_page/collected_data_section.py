import tkinter as tk


class CollectedDataSection(tk.Frame):
    """
    Main data display area showing collected demo data.
    """

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, bg="#CFCFCF")

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

        self.data_display = tk.Frame(
            self,
            bg="#EAEAEA",
            bd=1,
            relief="sunken",
        )
        self.data_display.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))

        tk.Label(
            self.data_display,
            text="Table/Graph of collected input and output data",
            bg="#EAEAEA",
            fg="#555",
        ).place(relx=0.5, rely=0.5, anchor="center")
