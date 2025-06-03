from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Button, Static
from rich.table import Table
from rich.panel import Panel

class MyApp(App):
    CSS = """
    Screen {
        align: center middle;
    }
    #button_bar {
        padding: 1 2;
        background: rgb(50,50,50);
    }
    Button {
        margin: 1;
        border: round cornflowerblue;
    }
    #table_panel {
        height: 10;
        width: 60;
        border: round green;
        padding: 1 2;
        background: rgb(20,20,20);
    }
    """

    def compose(self) -> ComposeResult:
        yield Static("[bold magenta]Rich Table Example[/bold magenta]\n", id="header")
        with Horizontal(id="button_bar"):
            yield Button("Show Fruits", id="btn_fruits")
            yield Button("Show Vegetables", id="btn_veggies")
            yield Button("Clear Table", id="btn_clear")
        yield Static(Panel("Press a button to show data...", title="Table Output"), id="table_panel")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        table_panel = self.query_one("#table_panel", Static)

        if event.button.id == "btn_fruits":
            table = Table(title="Fruits")
            table.add_column("Name", style="yellow", no_wrap=True)
            table.add_column("Color", style="green")
            table.add_column("Taste", style="magenta")
            table.add_row("Apple", "Red/Green", "Sweet")
            table.add_row("Banana", "Yellow", "Sweet")
            table.add_row("Lemon", "Yellow", "Sour")
            table.add_row("Grape", "Purple/Green", "Sweet/Sour")

            table_panel.update(Panel(table, title="Fruits Table"))

        elif event.button.id == "btn_veggies":
            table = Table(title="Vegetables")
            table.add_column("Name", style="yellow", no_wrap=True)
            table.add_column("Color", style="green")
            table.add_column("Type", style="cyan")
            table.add_row("Carrot", "Orange", "Root")
            table.add_row("Broccoli", "Green", "Flower")
            table.add_row("Spinach", "Green", "Leafy")
            table.add_row("Potato", "Brown", "Tuber")

            table_panel.update(Panel(table, title="Vegetables Table"))

        elif event.button.id == "btn_clear":
            table_panel.update(Panel("Table cleared.", title="Table Output"))

if __name__ == "__main__":
    MyApp().run()
