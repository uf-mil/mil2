import tkinter as tk


class CreateProjectPage(tk.Frame):
    def __init__(self, parent, controller=None):
        super().__init__(parent, bg="#DADADA")
        self.controller = controller

        label = tk.Label(self, text="Create Project", bg="#DADADA", fg="black")
        label.pack(anchor="w", padx=12, pady=10)
