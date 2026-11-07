"""Runs a node until it is told to stop."""

import signal
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def run_until_stopped(node_type):
    """
    Spin a node until SIGINT or SIGTERM, then shut it down.

    Ctrl-C reaches a launched node twice, from the terminal and again from launch,
    so the first signal only sets a flag and repeats are ignored: neither can land
    inside the cleanup, or kill the process while Python is exiting.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()

    def request_stop(signal_number, _frame):
        stop.set()
        signal.signal(signal_number, signal.SIG_IGN)

    for stop_signal in (signal.SIGINT, signal.SIGTERM):
        signal.signal(stop_signal, request_stop)

    node = node_type()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
