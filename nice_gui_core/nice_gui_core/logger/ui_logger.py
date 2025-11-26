from nicegui import Client, ui

from .logger import Logger


class UILogger(Logger):
    """UI Logger class."""

    def __init__(self, name: str) -> None:
        """Initialize the UILogger with a name.
        Args:
            name (str): The name of the logger.
        """
        super().__init__(name)
        self._client = None

    def set_client(self, client: Client) -> None:
        """Set the NiceGUI client.
        Args:
            client (Client): The NiceGUI client instance.
        """
        self._client = client

    def info(self, msg: str) -> None:
        """Log an info message.
        Args:
            msg (str): The message to log.
        """
        self._write(msg, "info")

    def warn(self, msg: str) -> None:
        """Log a warning message.
        Args:
            msg (str): The message to log.
        """
        self._write(msg, "warning")

    def error(self, msg: str) -> None:
        """Log an error message.
        Args:
            msg (str): The message to log.
        """
        self._write(msg, "negative")

    def debug(self, msg: str) -> None:
        """Log a debug message.
        Args:
            msg (str): The message to log.
        """
        pass

    def positive(self, msg: str) -> None:
        """Log a positive message.
        Args:
            msg (str): The message to log.
        """
        self._write(msg, "positive")

    def _write(self, msg: str, type_str: str) -> None:
        """Write a message to the UI.
        Args:
            msg (str): The message to log.
            type_str (str): The type of the message (info, warning, negative, positive).
        """
        if self._client is not None:
            with self._client:
                ui.notify(self._return_msg(msg), type=type_str)
