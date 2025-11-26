from nicegui import ui


class UINotificationHandler():
    def __init__(self):
        self._notification = None

    def display(self, msg: str):
        if self._notification is None:
            self._notification = ui.notification(timeout=None)
        self._notification.message = msg

    def dismiss(self):
        if not self._notification is None:
            self._notification.dismiss()
        self._notification = None
