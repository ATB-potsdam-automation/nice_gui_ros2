from typing import Any, Callable

from rclpy.node import Node
from rclpy.subscription import Subscription


class SubscriberHandler:
    """Class to handle ROS2 subscribers"""

    def __init__(
        self,
        node: Node,
        msg_type: Any,
        topic_name: str,
        callback: Callable,
        queue_size: int = 1,
    ):
        """Initialize the subscriber handler
        Args:
            node (Node): the ROS2 node
            msg_type (Any): the message type
            topic_name (str): the topic name
            callback (Callable): the callback function
            queue_size (int): the queue size
        """
        self._node = node
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._callback = callback
        self._queue_size = queue_size

        self._subscriber: Subscription | None = None

    def is_active(self) -> bool:
        """Check if the subscriber is active
        Returns:
            bool: True if the subscriber is active, False otherwise
        """
        is_active = False
        if self._subscriber is not None:
            is_active = True
        return is_active

    def subscribe(self) -> None:
        """Subscribe to the topic"""
        if self._subscriber is None:
            self._node.get_logger().debug(f"Subscribe {self._topic_name}")
            self._subscriber = self._node.create_subscription(
                self._msg_type, self._topic_name, self._callback, self._queue_size
            )

    def unsubscribe(self) -> None:
        """Unsubscribe from the topic"""
        if self._subscriber is not None:
            self._node.get_logger().debug(f"Unsubscribe {self._topic_name}")
            self._node.destroy_subscription(self._subscriber)
            self._subscriber = None
