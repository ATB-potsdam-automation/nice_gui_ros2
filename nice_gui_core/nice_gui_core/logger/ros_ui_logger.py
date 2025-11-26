from rclpy.logging import RcutilsLogger

from .logger import Logger
from .ros_logger import RosLogger
from .ui_logger import UILogger


class RosUILogger(Logger):
    """Combined ROS and UI Logger class."""

    def __init__(self, name: str, ros_logger: RcutilsLogger) -> None:
        """Initialize the RosUILogger with a name and a ROS logger.
        Args:
            name (str): The name of the logger.
            ros_logger (RcutilsLogger): The ROS logger instance.
        """
        self._ros_logger = RosLogger(name, ros_logger)
        self._ui_logger = UILogger(name)

    def set_client(self, client) -> None:
        """Set the NiceGUI client.
        Args:
            client (Client): The NiceGUI client instance.
        """
        self._ui_logger.set_client(client)

    def info(self, msg: str) -> None:
        """Log an info message.
        Args:
            msg (str): The message to log.
        """
        self._ros_logger.info(msg)
        self._ui_logger.info(msg)

    def warn(self, msg: str) -> None:
        """Log a warning message.
        Args:
            msg (str): The message to log.
        """
        self._ros_logger.warn(msg)
        self._ui_logger.warn(msg)

    def error(self, msg: str) -> None:
        """Log an error message.
        Args:
            msg (str): The message to log.
        """
        self._ros_logger.error(msg)
        self._ui_logger.error(msg)

    def debug(self, msg: str) -> None:
        """Log a debug message.
        Args:
            msg (str): The message to log.
        """
        self._ros_logger.debug(msg)
        self._ui_logger.debug(msg)

    def positive(self, msg: str) -> None:
        """Log a positive message.
        Args:
            msg (str): The message to log.
        """
        self._ros_logger.positive(msg)
        self._ui_logger.positive(msg)
