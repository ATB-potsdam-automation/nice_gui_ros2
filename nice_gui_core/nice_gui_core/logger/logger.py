class Logger:
    """Base Logger class."""

    def __init__(self, name: str) -> None:
        """Initialize the Logger with a name.
        Args:
            name (str): The name of the logger.
        """
        self._name = name

    def info(self, msg: str) -> None:
        """Log an info message.
        Args:
            msg (str): The message to log.
        """
        pass

    def warn(self, msg: str) -> None:
        """Log a warning message.
        Args:
            msg (str): The message to log.
        """
        pass

    def error(self, msg: str) -> None:
        """Log an error message.
        Args:
            msg (str): The message to log.
        """
        pass

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
        pass

    def _return_msg(self, msg: str) -> str:
        """Return the formatted log message.
        Args:
            msg (str): The message to format.
        Returns:
            str: The formatted message.
        """
        return f"[{self._name}] {msg}"

    def get_logger(self, name: str) -> "LoggerAdapter":
        """Return a lightweight adapter that always adds `name` to messages.

        The adapter delegates to this logger's methods but prefixes every
        message with the provided name so callers don't have to repeat it.
        This is cheap and keeps a single underlying logger instance.
        """
        return LoggerAdapter(self, name)


class LoggerAdapter(Logger):
    def __init__(self, parent_logger: Logger, name: str) -> None:
        self._parent = parent_logger
        self._name = name

    def _prefix(self, msg: str) -> str:
        # Always include the forced name first, then let the parent
        # format the rest (parent may add its own name/caller info).
        return f"[{self._name}] {msg}"

    def info(self, msg: str) -> None:
        self._parent.info(self._prefix(msg))

    def warn(self, msg: str) -> None:
        self._parent.warn(self._prefix(msg))

    def error(self, msg: str) -> None:
        self._parent.error(self._prefix(msg))

    def debug(self, msg: str) -> None:
        self._parent.debug(self._prefix(msg))

    def positive(self, msg: str) -> None:
        self._parent.positive(self._prefix(msg))

    def return_msg(self, msg: str) -> str:
        # Preserve compatibility for callers that use return_msg
        return self._parent._return_msg(self._prefix(msg))
