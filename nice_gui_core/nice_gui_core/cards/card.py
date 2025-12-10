from typing import List

from nice_gui_core.logger import RosUILogger
from nice_gui_core.utils import ClientHandler, get_parameter
from rclpy.node import Node
from rclpy.parameter import Parameter


class Card:
    """Base class for NiceGUI cards"""

    def __init__(self, node: Node, param_prefix: str):
        """Initialize the NiceGUI card
        Args:
            node (Node): the ROS2 node
            param_prefix (str): the parameter prefix for this card
        """
        self._node = node
        self._client_handler = ClientHandler(node)
        self._active_clients = 0
        self._name = get_parameter(
            node, param_prefix + ".name", Parameter.Type.STRING
        ).string_value
        self._logger = RosUILogger(self._name, node.get_logger())

    def create_card(self):
        """Create the NiceGUI card and set up the client"""
        self._client_handler.create_client()
        self._logger.set_client(self._client_handler.get_client())
