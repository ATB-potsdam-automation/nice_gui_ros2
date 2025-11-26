from rclpy.logging import RcutilsLogger

from .logger import Logger


class RosLogger(Logger):
    """ROS Logger class."""

    def __init__(self, name: str, logger: RcutilsLogger) -> None:
        """Initialize the RosLogger with a name and a ROS logger.
        Args:
            name (str): The name of the logger.
            logger (RcutilsLogger): The ROS logger instance.
        """
        super().__init__(name)
        self._logger = logger

    def warn(self, msg: str) -> None:
        """Log a warning message.
        Args:
            msg (str): The message to log.
        """
        self._logger.warn(self._return_msg(msg))

    def info(self, msg: str) -> None:
        """Log an info message.
        Args:
            msg (str): The message to log.
        """
        self._logger.info(self._return_msg(msg))

    def error(self, msg: str) -> None:
        """Log an error message.
        Args:
            msg (str): The message to log.
        """
        self._logger.error(self._return_msg(msg))

    def debug(self, msg: str) -> None:
        """Log a debug message.
        Args:
            msg (str): The message to log.
        """
        self._logger.debug(self._return_msg(msg))

    def positive(self, msg: str) -> None:
        """Log a positive message.
        Args:
            msg (str): The message to log.
        """
        self._logger.info(self._return_msg(msg))
