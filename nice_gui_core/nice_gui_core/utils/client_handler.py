from typing import List

from nicegui import context
from rclpy.node import Node, Timer

from .subscriber_handler import SubscriberHandler


class ClientHandler:
    """Class to handle NiceGUI clients"""

    def __init__(self, node: Node):
        """Initialize the client handler
        Args:
            node (Node): the ROS2 node
        """
        self._node = node
        self._logger = node.get_logger()

        self._subscriber: List[SubscriberHandler] = []
        self._timer: List[Timer] = []

        self._client = None
        self._number_of_clients = 0

    def add_subscriber(self, subscriber: SubscriberHandler):
        """Add a subscriber to be managed by the client handler
        Args:
            subscriber (SubscriberHandler): the subscriber to add
        """
        self._subscriber.append(subscriber)

    def add_timer(self, timer: Timer):
        """Add a timer to be managed by the client handler
        Args:
            timer (Timer): the timer to add
        """
        self._timer.append(timer)

    def create_client(self):
        """Create the NiceGUI client and set up connect/disconnect callbacks"""
        self._client = context.client

        def on_client_disconnect():
            self._number_of_clients -= 1
            self._logger.debug(
                f"Client disconnected. Clients left: {self._number_of_clients}"
            )

            if self._number_of_clients == 0:
                for subscriber in self._subscriber:
                    subscriber.unsubscribe()
                for timer in self._timer:
                    if timer.is_ready():
                        timer.cancel()

        def on_client_connect():
            self._number_of_clients += 1
            for subscriber in self._subscriber:
                subscriber.subscribe()
            for timer in self._timer:
                timer.reset()

        self._client.on_disconnect(on_client_disconnect)
        self._client.on_connect(on_client_connect)

    def get_client(self):
        """Get the NiceGUI client
        Returns:
            The NiceGUI client
        """
        return self._client
