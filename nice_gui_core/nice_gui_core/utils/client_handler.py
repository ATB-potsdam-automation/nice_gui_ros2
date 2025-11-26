from typing import List

from nicegui import context
from rclpy.node import Node, Timer

from .subscriber_handler import SubscriberHandler


class ClientHandler():
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        self._subscriber: List[SubscriberHandler] = []
        self._timer: List[Timer] = []

        self._client = None
        self._number_of_clients = 0

    def add_subscriber(self, subscriber: SubscriberHandler):
        self._subscriber.append(subscriber)

    def add_timer(self, timer: Timer):
        self._timer.append(timer)

    def create_client(self):
        self._client = context.client

        def on_client_disconnect():
            self._number_of_clients -= 1
            self._logger.debug(f"Client disconnected. Clients left: {self._number_of_clients}")

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
        return self._client
