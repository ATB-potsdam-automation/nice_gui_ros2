from nicegui import ui


class UINotificationHandler:
    """Class to handle UI notifications"""

    def __init__(self):
        self._notification = None

    def display(self, msg: str):
        """Display a notification with the given message
        Args:
            msg (str): the message to display
        """
        if self._notification is None:
            self._notification = ui.notification(timeout=None)
        self._notification.message = msg

    def dismiss(self):
        """Dismiss the current notification"""
        if not self._notification is None:
            self._notification.dismiss()
        self._notification = None
