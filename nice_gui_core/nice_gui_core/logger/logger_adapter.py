from .logger import Logger


class LoggerAdapter(Logger):
    """A lightweight adapter that always adds a forced name to messages."""

    def __init__(self, parent_logger: Logger, forced_name: str) -> None:
        """Initialize the adapter."""
        self._parent = parent_logger
        self._forced_name = forced_name

    def _prefix(self, msg: str) -> str:
        """Prefix the message with the forced name.
        Args:
            msg (str): The message to prefix.
        Returns:
            str: The prefixed message.
        """
        # Always include the forced name first, then let the parent
        # format the rest (parent may add its own name/caller info).
        return f"[{self._forced_name}] {msg}"

    def info(self, msg: str) -> None:
        """Log an info message.
        Args:
            msg (str): The message to log.
        """
        self._parent.info(self._prefix(msg))

    def warn(self, msg: str) -> None:
        """Log a warning message.
        Args:
            msg (str): The message to log.
        """
        self._parent.warn(self._prefix(msg))

    def error(self, msg: str) -> None:
        """Log an error message.
        Args:
            msg (str): The message to log.
        """
        self._parent.error(self._prefix(msg))

    def debug(self, msg: str) -> None:
        """Log a debug message.
        Args:
            msg (str): The message to log.
        """
        self._parent.debug(self._prefix(msg))

    def positive(self, msg: str) -> None:
        """Log a positive message.
        Args:
            msg (str): The message to log.
        """
        self._parent.positive(self._prefix(msg))

    def return_msg(self, msg: str) -> str:
        """Return the formatted log message.
        Args:
            msg (str): The message to format.
        Returns:
            str: The formatted message.
        """
        # Preserve compatibility for callers that use return_msg
        return self._parent._return_msg(self._prefix(msg))
