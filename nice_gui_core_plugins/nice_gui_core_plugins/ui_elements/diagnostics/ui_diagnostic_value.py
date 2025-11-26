from diagnostic_msgs.msg import KeyValue
from nicegui import ui


class UIDiagnosticValue:
    def __init__(self, key_value: KeyValue):
        self._key = key_value.key
        self._value = key_value.value
        self._ui_created = False

    def create_ui(self):
        self._ui_created = True
        with ui.item():
            with ui.item_section():
                self._ui_key = ui.item_label(self._key)
            with ui.item_section():
                self._ui_value = ui.item_label(self._value)

    def update_ui(self):
        if not self._ui_created:
            self.create_ui()
        self._ui_value.set_text(self._value)

    def update_value(self, value: str):
        self._value = value
