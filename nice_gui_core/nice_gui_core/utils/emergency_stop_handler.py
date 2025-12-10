from typing import Callable

from nicegui import ui
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


class EmergencyStopButton:
    """Base class for emergency stop button"""

    def __init__(self, fun_set_estop: Callable) -> None:
        """Initialize the emergency stop button
        Args:
            fun_set_estop: function to set the emergency stop
        """
        self.icon_container = None
        self.icon = None
        self.icon_tooltip = None
        self.icon_label = None
        self._fun_set_estop = fun_set_estop

    def create_button(self, value: bool):
        """Create the emergency stop button
        Args:
            value (bool): whether the button is visible or not
        """
        with ui.button(
            "", on_click=lambda: self.on_click_emergency_stop_button()
        ) as self.icon_container:
            with ui.column().classes("items-center"):
                self.icon = ui.label("")
                with self.icon:
                    self.icon_tooltip = ui.tooltip()

                # Make the message appear below with some margin
                self.icon_label = ui.label("EMERGENCY STOP")
        self.set_icon_props()

    def on_click_emergency_stop_button(self):
        """Callback when the emergency stop button is clicked"""
        if self._fun_set_estop():
            ui.notify("Set Nice Gui emergency stop!")
        else:
            ui.notify("Reset Nice Gui emergency stop!")

    def set_visible(self, value: bool):
        """Set the visibility of the emergency stop button
        Args:
            value (bool): whether the button is visible or not
        """
        if self.icon_container is not None:
            self.icon_container.set_visibility(value)

    def set_tooltip_text(self, text: str):
        """Set the tooltip text of the emergency stop button
        Args:
            text (str): the tooltip text
        """
        if self.icon is not None:
            self.icon_tooltip.set_text(text)


class EmergencyStopButtonTriggered(EmergencyStopButton):
    def __init__(self, fun_estop):
        super().__init__(fun_estop)

    def set_icon_props(self):
        self.icon_label.classes("text-red-600 font-bold text-lg text-center")
        self.icon.classes(
            "bg-red-600 rounded-full w-20 h-20 flex items-center justify-center"
        )
        self.icon_container.classes(
            "fixed bottom-4 right-4 z-50 animate-pulse bg-transparent"
        )


class EmergencyStopButtonNotTriggered(EmergencyStopButton):
    def __init__(self, fun_estop):
        super().__init__(fun_estop)

    def set_icon_props(self):
        self.icon_label.classes("text-gray-600 font-bold text-lg text-center")
        self.icon.classes(
            "bg-gray-600 rounded-full w-20 h-20 flex items-center justify-center"
        )
        self.icon_container.classes("fixed bottom-4 right-4 z-50 bg-transparent")
        self.icon_tooltip.set_visibility(False)


class EmergencyStopHandler:
    def __init__(self, node: Node):
        self._emergency_stop = False
        self._emergency_stop_nice_gui = False
        self._logger = node.get_logger()
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._sub_emergency_stop = node.create_subscription(
            Bool, "/emergency_stop/global", self.callback_emergency_stop, qos_profile
        )
        self._sub_emergency_stop_message = node.create_subscription(
            String,
            "/emergency_stop/message",
            self.callback_emergency_stop_message,
            qos_profile,
        )

        self.pub_emergency_stop_nice_gui = node.create_publisher(
            Bool, "/emergency_stop/nice_gui", 1
        )

        self.pub_emergency_stop_nice_gui.publish(Bool(data=False))
        self._emergency_stop_message = ""

        self._emergency_stop_button_triggered = EmergencyStopButtonTriggered(
            self.set_emergency_stop_nice_gui
        )
        self._emergency_stop_button_not_triggered = EmergencyStopButtonNotTriggered(
            self.set_emergency_stop_nice_gui
        )

    def create_notification(self):
        self._emergency_stop_button_triggered.create_button(False)
        self._emergency_stop_button_not_triggered.create_button(True)
        self.set_emergency_stop_button()
        self._emergency_stop_button_triggered.set_tooltip_text(
            self._emergency_stop_message
        )

    def set_emergency_stop_nice_gui(self):
        self._logger.debug("Emergency stop triggered by ui")
        self._emergency_stop_nice_gui = not self._emergency_stop_nice_gui
        msg = Bool()
        msg.data = self._emergency_stop_nice_gui
        self.pub_emergency_stop_nice_gui.publish(msg)
        return self._emergency_stop_nice_gui

    def set_emergency_stop_button(self):
        if self._emergency_stop:
            self._emergency_stop_button_triggered.set_visible(True)
            self._emergency_stop_button_not_triggered.set_visible(False)
        else:
            self._emergency_stop_button_triggered.set_visible(False)
            self._emergency_stop_button_not_triggered.set_visible(True)

    def callback_emergency_stop(self, msg: Bool):
        self._emergency_stop = msg.data
        self.set_emergency_stop_button()

    def callback_emergency_stop_message(self, msg: String):
        self._emergency_stop_message = msg.data
        self._emergency_stop_button_triggered.set_tooltip_text(
            self._emergency_stop_message
        )
        )
