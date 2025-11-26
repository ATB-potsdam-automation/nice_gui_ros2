from typing import Callable

from nice_gui_core.logger import Logger
from rclpy.node import Node


class StateMachine:
    def __init__(self, node: Node, logger: Logger) -> None:
        """Base class for NiceGUI state machines
        Args:
            node (Node): the ROS2 node
            logger (Logger): the logger instance
        """
        self._node = node
        self._logger = logger
        self._cancel_requested = False

    def _init_transitions(self) -> None:
        """Initialize the state machine transitions"""
        pass

    def start(self) -> None:
        """Start state machine"""
        pass

    def cancel(self) -> None:
        """Cancel state machine"""
        pass

    def set_on_succeeded(self, callback: Callable) -> None:
        """Set the callback function when the state machine succeeded
        Args:
            callback (Callable): the callback function
        """
        self._on_succeeded = callback

    def set_on_aborted(self, callback: Callable) -> None:
        """Set the callback function when the state machine aborts
        Args:
            callback (Callable): the callback function
        """
        self._on_aborted = callback

    def set_on_canceled(self, callback: Callable) -> None:
        """Set the callback function when the state machine cancels
        Args:
            callback (Callable): the callback function
        """
        self._on_canceled = callback

    def _on_succeeded(self) -> None:
        """Callback when the state machine succeeded"""
        pass

    def _on_aborted(self) -> None:
        """Callback when the state machines aborts"""
        pass

    def _on_canceled(self) -> None:
        """Callback when the state machines cancels"""
        pass
