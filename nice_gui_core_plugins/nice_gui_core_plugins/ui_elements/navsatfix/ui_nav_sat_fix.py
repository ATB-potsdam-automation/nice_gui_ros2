from nicegui import ui
from sensor_msgs.msg import NavSatFix


class UINavSatFix:
    def __init__(self, name: str):
        self._navsatfix = NavSatFix()
        self._ui_created = False
        self._name = name

    def create_ui(self):
        with ui.item():
            with ui.item_section().props("avatar"):
                self._ui_status = ui.label(self.get_status())
            with ui.item_section():
                self._ui_label = ui.item_label(self._name)
            with ui.item_section().props("side"):
                self.create_dialog()

    def create_dialog(self):
        self._ui_dialog = ui.dialog()
        with self._ui_dialog as dialog, ui.card():
            with ui.list().props("bordered separator"):
                self.create_table()
            ui.button("Close", on_click=dialog.close)
        ui.button(icon="chat", on_click=dialog.open)

    def create_table(self):
        self._ui_created = True
        columns = [
            {
                "name": "name",
                "label": "Name",
                "field": "name",
                "required": True,
                "align": "left",
            },
            {"name": "value", "label": "Value", "field": "value", "sortable": False},
        ]
        self._table_rows = [
            {"name": "Status", "value": self.get_status()},
            {"name": "Latitude", "value": self._navsatfix.latitude},
            {"name": "Longitude", "value": self._navsatfix.longitude},
            {"name": "Altitude", "value": self._navsatfix.altitude},
        ]
        self._table = ui.table(columns=columns, rows=self._table_rows, row_key="name")

    def update_value(self, value: NavSatFix):
        self._navsatfix = value

    def update_ui(self):
        if not self._ui_created:
            self.create_ui()
        self._ui_status.set_text(self.get_status())
        self._table_rows[0]["value"] = self.get_status()
        self._table_rows[1]["value"] = self._navsatfix.latitude
        self._table_rows[2]["value"] = self._navsatfix.longitude
        self._table_rows[3]["value"] = self._navsatfix.altitude

        self._table.update_rows(self._table_rows)

    def get_status(self) -> str:
        status = self._navsatfix.status.status
        if status == -1:
            return "No Fix"
        elif status == 0:
            return "Fix"
        elif status == 1:
            return "SBAS"
        elif status == 2:
            return "GBAS"
        else:
            return "Unknown"
