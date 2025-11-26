from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler, get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState

STATUS_MAP = {
    value: name.replace("POWER_SUPPLY_STATUS_", "")
    for name, value in vars(BatteryState).items()
    if name.startswith("POWER_SUPPLY_STATUS_")
}


def status_to_string(status_value: int) -> str:
    return STATUS_MAP.get(status_value, f"INVALID({status_value})")


class BatteryStateCard(Card):

    def __init__(self, node: Node, param_prefix: str) -> None:
        super().__init__(node, param_prefix)
        self._battery_state = BatteryState()
        topic = get_parameter(
            node, param_prefix + ".topic", Parameter.Type.STRING
        ).string_value
        self._client_handler.add_subscriber(
            SubscriberHandler(node, BatteryState, topic, self.update_values)
        )

    def create_card(self) -> None:
        super().create_card()
        with ui.card().classes("w-54 text-center items-center"):
            ui.label(self._name).classes("text-2xl")

            with ui.grid().classes("w-full"):
                self.create_table()

    def create_table(self):
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
            # {'name': 'Status', 'value': status_to_string(self._battery_state.power_supply_status)},
            {
                "name": "Percentage",
                "value": f"{100 * self._battery_state.percentage:.2f}%",
            },
            {"name": "Voltage", "value": f"{self._battery_state.voltage:.2f}"},
            {"name": "Current", "value": f"{self._battery_state.current:.2f}"},
            {"name": "Temperature", "value": f"{self._battery_state.temperature:.2f}"},
        ]
        self._table = ui.table(columns=columns, rows=self._table_rows, row_key="name")

    def update_table(self):
        # self._table_rows[0]['value'] = status_to_string(self._battery_state.power_supply_status)
        self._table_rows[0]["value"] = f"{100 * self._battery_state.percentage:.2f}%"
        self._table_rows[1]["value"] = f"{self._battery_state.voltage:.2f}"
        self._table_rows[2]["value"] = f"{self._battery_state.current:.2f}"
        self._table_rows[3]["value"] = f"{self._battery_state.temperature:.2f}"

        self._table.update_rows(self._table_rows)

    def update_values(self, msg: BatteryState):
        self._battery_state = msg
        self.update_table()
