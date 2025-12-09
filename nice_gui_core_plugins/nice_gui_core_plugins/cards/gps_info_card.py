from functools import partial
from typing import List

from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler, get_parameter
from nice_gui_core_plugins.ui_elements.navsatfix.ui_nav_sat_fix import UINavSatFix
from nicegui import context, ui
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix


class GPSInfoCard(Card):
    def __init__(self, node: Node, param_prefix: str):
        super().__init__(node, param_prefix)

        gps_subscriber_topics: List[str] = get_parameter(
            node, param_prefix + ".gps_topics", Parameter.Type.STRING_ARRAY
        ).string_array_value
        self._ui_navsatfix_handler: List[UINavSatFix] = []
        for topic in gps_subscriber_topics:
            ui_navsatfix = UINavSatFix(topic)
            callback = partial(self.gps_callback, ui_navsatfix=ui_navsatfix)
            self._client_handler.add_subscriber(
                SubscriberHandler(node, NavSatFix, topic, callback)
            )
            self._ui_navsatfix_handler.append(ui_navsatfix)

    def gps_callback(self, msg: NavSatFix, ui_navsatfix: UINavSatFix) -> None:
        ui_navsatfix.update_value(msg)
        self.update_ui()

    def create_card(self):
        super().create_card()
        with ui.card().classes("max-w-96 max-h-110 items-center"):
            ui.label(self._name).classes("text-2xl")
            self.ui_list = ui.list().props("bordered separator")
            with self.ui_list:
                for ui_handler in self._ui_navsatfix_handler:
                    ui_handler.create_ui()
                    # Create initial entries

    def update_ui(self):
        with self._client_handler.get_client():
            for ui_handler in self._ui_navsatfix_handler:
                ui_handler.update_ui()
