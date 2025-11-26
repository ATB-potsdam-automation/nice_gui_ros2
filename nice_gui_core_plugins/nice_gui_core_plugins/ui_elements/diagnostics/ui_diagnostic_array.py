from typing import Dict

from nicegui import ui

from .ui_diagnostic_status import UIDiagnosticStatus


class UIDiagnosticArray:
    def __init__(self):
        self._ui_diagnostic_status_dict: Dict[str, UIDiagnosticStatus] = {}

    def create_ui(self):
        self.ui_list = ui.list().props("bordered separator")
        with self.ui_list:
            # Create initial entries
            for ui_diagnostic_status in self._ui_diagnostic_status_dict.values():
                ui_diagnostic_status.create_ui()

    def update_ui(self):
        with self.ui_list:
            for ui_diagnostic_status in self._ui_diagnostic_status_dict.values():
                ui_diagnostic_status.update_ui()

    def update_data(self, array):
        for status in array.status:
            if status.name in self._ui_diagnostic_status_dict:
                # Update existing entry
                self._ui_diagnostic_status_dict[status.name].update_status(status)
            else:
                self._ui_diagnostic_status_dict[status.name] = UIDiagnosticStatus(
                    status
                )

    def filter_out_ok(self):
        for ui_diagnostic_status in self._ui_diagnostic_status_dict.values():
            if ui_diagnostic_status.level != 0:
                ui_diagnostic_status.set_visibility(True)
            else:
                ui_diagnostic_status.set_visibility(False)
