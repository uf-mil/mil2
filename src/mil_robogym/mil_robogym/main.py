import rclpy

from mil_robogym.ui.app import App


def main():
    rclpy.init()
    app = App()
    app.mainloop()
