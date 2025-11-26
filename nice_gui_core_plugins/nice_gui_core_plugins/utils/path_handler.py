from rclpy.node import Node
from nav_msgs.msg import Path
from typing import Callable, List


class PathHandler():
    def __init__(self, node: Node, topic_name: str):
        self._path = None
        self._path_observers: List[Callable[[Path], None]] = []
        self._sub_path = node.create_subscription(
            Path, topic_name, self.path_callback, 1)

    def register_observer(self, path: Callable[[Path], None]):
        self._path_observers.append(path)

    def path_callback(self, msg: Path):
        self.update_path(msg)

    def update_path(self, path: Path | None) -> None:
        self._path = path

        for path_observer in self._path_observers:
            path_observer(path)

    def delete_path(self):
        self.update_path(None)
