from typing import Callable

from rclpy.node import Node


class StateWait:
    def __init__(self, node: Node, wait_time: float = 1.0):
        """Initialize the wait state machine
        Args:
            node (Node): the ROS2 node
            wait_time (float): the wait time in seconds
        """
        self._node = node
        self._wait_time = wait_time
        self._timer_wait = None

    def set_wait_time(self, wait_time: float):
        """Set the wait time in seconds"""
        self._wait_time = wait_time

    def start(self):
        """Start the wait state machine"""
        self._timer_wait = self._node.create_timer(self._wait_time, self._timer_done)

    def cancel(self):
        """Cancel the wait state machine"""
        if self._timer_wait is not None:
            self._timer_wait.cancel()

    def _timer_done(self):
        """Callback when the wait is over"""
        self._timer_wait.cancel()
        self._on_succeeded()

    def set_on_succeeded(self, callback: Callable):
        """Set the callback function when the wait is over
        Args:
            callback (Callable): the callback function
        """
        self._on_succeeded = callback

    def _on_succeeded(self):
        pass
