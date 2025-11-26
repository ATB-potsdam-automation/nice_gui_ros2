from typing import Dict, List

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nicegui import ui

from .ui_diagnostic_value import UIDiagnosticValue


class UIDiagnosticStatus:
    """
    Class to represent a single diagnostic entry.
    """

    def __init__(self, status: DiagnosticStatus) -> None:
        self.name = status.name
        self.ui_up_to_date: bool = False
        self._values: Dict[str, UIDiagnosticValue] = {}
        self.update_status(status)
        self._ui_created = False

    def create_ui(self) -> None:
        self.ui_up_to_date = True
        self._ui_created = True
        self._item = ui.item()
        with self._item:
            with ui.item_section().props("avatar"):
                self._ui_level = ui.item_label(self.level)
                self.set_level(self._ui_level)
            with ui.item_section().classes("max-w-32"):
                self._ui_label = ui.item_label(self.name)
            with ui.item_section().props("side"):
                self.create_dialog()

    def update_ui(self) -> None:
        """
        Update the diagnostic entry with a new level.
        """
        if self.ui_up_to_date:
            return

        if not self._ui_created:
            self.create_ui()

        self.set_level(self._ui_level)
        self.set_level(self._ui_dialog_level)
        self.ui_dialog_message.set_text(f"Message: {self.status.message}")
        self.update_ui_values()
        self.ui_up_to_date = True

    def update_status(self, status: DiagnosticStatus):
        self.ui_up_to_date = False
        self.status = status
        self.level = int.from_bytes(status.level, byteorder="little")
        self.update_values(status.values)

    def update_values(self, values: List[KeyValue]):
        for key_value in values:
            if key_value.key not in self._values:
                self._values[key_value.key] = UIDiagnosticValue(key_value)
            else:
                self._values[key_value.key].update_value(key_value.value)

    def update_ui_values(self) -> None:
        with self.ui_value_list:
            # Create initial entries
            for _, value in self._values.items():
                value.update_ui()

    def create_dialog(self) -> None:
        self._ui_dialog = ui.dialog()
        with self._ui_dialog as dialog, ui.card():
            self._ui_dialog_header = ui.label(self.name).classes("text-2xl")
            self._ui_dialog_level = ui.label()
            self.set_level(self._ui_dialog_level)
            self.ui_dialog_message = ui.label(f"Message: {self.status.message}")
            self.ui_value_list = ui.list().props("bordered separator")
            with self.ui_value_list:
                ui.item_label("Values").props("header").classes("text-bold")
                ui.separator()
                for _, value in self._values.items():
                    value.create_ui()
            ui.button("Close", on_click=dialog.close)
        ui.button(icon="chat", on_click=dialog.open)

    def set_level(self, label: ui.label):
        if self.level == 0:
            label.set_text("OK")
            label.style("color: green")
        elif self.level == 1:
            label.set_text("WARN")
            label.style("color: orange")
        else:
            label.set_text("ERROR")
            label.style("color: red")

    def set_visibility(self, visbility: bool) -> None:
        self._item.set_visibility(visbility)
