from typing import Any, Callable

from rclpy.node import Node
from rclpy.subscription import Subscription


class SubscriberHandler:
    def __init__(self, node: Node, msg_type: Any, topic_name: str,
                 callback: Callable, queue_size: int = 1):
        self._node = node
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._callback = callback
        self._queue_size = queue_size

        self._subscriber: Subscription | None = None

    def is_active(self) -> bool:
        is_active = False
        if self._subscriber is not None:
            is_active = True
        return is_active

    def subscribe(self):
        if self._subscriber is None:
            self._node.get_logger().debug(f"Subscribe {self._topic_name}")
            self._subscriber = self._node.create_subscription(
                self._msg_type, self._topic_name, self._callback, self._queue_size)

    def unsubscribe(self):
        if self._subscriber is not None:
            self._node.get_logger().debug(f"Unsubscribe {self._topic_name}")
            self._node.destroy_subscription(self._subscriber)
            self._subscriber = None
