from diagnostic_msgs.msg import DiagnosticArray
from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler
from nice_gui_core_plugins.ui_elements.diagnostics import UIDiagnosticArray
from nicegui import context, ui
from rclpy.node import Node


class DiagnosticCard(Card):
    """
    Card to display ROS 2 diagnostic messages in real time.
    """

    def __init__(self, node: Node, param_prefix: str) -> None:
        """
        Initialize the diagnostic card and set up ROS 2 subscriber.
        """
        super().__init__(node, param_prefix)
        # Holds latest diagnostic messages
        self._ui_diagnostic_array_list = UIDiagnosticArray()
        self._ui_diagnostic_array_dialog = UIDiagnosticArray()

        self.ui_list = None  # Will reference the UI list element

        self._client_handler.add_timer(
            self._node.create_timer(1.0, self.update_ui, autostart=False)
        )  # Update UI every second

        self._client_handler.add_subscriber(
            SubscriberHandler(
                node, DiagnosticArray, "/diagnostics", self.diagnostic_callback
            )
        )

    def diagnostic_callback(self, diagnostic_array: DiagnosticArray) -> None:
        """
        Callback to process incoming diagnostics and update UI.
        """

        self._ui_diagnostic_array_list.update_data(diagnostic_array)
        self._ui_diagnostic_array_dialog.update_data(diagnostic_array)

    def update_ui(self) -> None:
        """
        Refresh the list display in the card.
        """
        with self._client_handler.get_client():
            self._ui_diagnostic_array_list.update_ui()
            self._ui_diagnostic_array_list.filter_out_ok()
            self._ui_diagnostic_array_dialog.update_ui()

    def create_card(self) -> None:
        """
        Create the visual diagnostic card.
        """
        super().create_card()
        with ui.card().classes("text-left items-start"):
            ui.label(self._name).classes("text-2xl")
            self.create_list()
            self.create_dialog_all()

    def create_list(self) -> None:
        """
        Create the list to display diagnostic messages.
        """
        self._ui_diagnostic_array_list.create_ui()
        self._ui_diagnostic_array_list.filter_out_ok()

    def create_dialog_all(self) -> None:
        self._ui_dialog = ui.dialog()
        with self._ui_dialog as dialog, ui.card():
            self._ui_diagnostic_array_dialog.create_ui()
            ui.button("Close", on_click=dialog.close)
        ui.button(text="See all", on_click=dialog.open)
