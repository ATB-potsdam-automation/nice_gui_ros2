import importlib

from nice_gui_core.utils import EmergencyStopHandler, get_parameter, load_plugins
from nicegui import Client, ui
from rclpy.node import Node
from rclpy.parameter import Parameter

# MODULE_CARDS = importlib.import_module("nice_gui_core.cards")

CARD_PLUGINS = load_plugins("nice_gui.cards")


class PageWithCards:
    """Class for NiceGUI pages with cards"""

    def __init__(self, node: Node, page_name: str) -> None:
        """Initialize the NiceGUI page with cards
        Args:
            node (Node): the ROS2 node
            page_name (str): the page name
        """
        self.card_names = get_parameter(
            node, page_name + ".cards", Parameter.Type.STRING_ARRAY
        ).string_array_value
        self._cards = []
        for card_name in self.card_names:
            card_type = get_parameter(
                node, page_name + "." + card_name + ".type", Parameter.Type.STRING
            ).string_value
            card = CARD_PLUGINS[card_type](node, page_name + "." + card_name)
            self._cards.append(card)

        # Create page
        url = get_parameter(
            node, page_name + ".url", Parameter.Type.STRING
        ).string_value
        node.get_logger().info(f"Creating page {page_name} with url {url}")

        show_emergency_stop = get_parameter(
            node, "emergency_stop.display", False
        ).bool_value
        if show_emergency_stop:
            self._emergency_stop_handler = EmergencyStopHandler(node)

        @ui.page(url)
        def page_function(client: Client) -> None:
            ui.colors(
                primary="#006DB5", secondary="#7FBA23"
            )  # set a custom primary color (e.g. blue)
            self.create_page(client)
            if show_emergency_stop:
                self._emergency_stop_handler.create_notification()

    def create_page(self, client: Client) -> None:
        """Create the NiceGUI page layout with cards
        Args:
            client (Client): the NiceGUI client
        """
        with ui.header().classes("items-center justify-between bg-white p-2 shadow-md"):
            # Left logo
            ui.image("/static/logo_atb_detail.png").classes(
                "h-15 w-55 mx-4 cursor-pointer scale-110"
            ).on("click", lambda: ui.navigate.to("/"))

            # Right logo
            ui.image("/static/logo_tu_berlin.png").classes("h-15 w-45 mx-4 scale-80")
        with ui.row().classes("items-stretch"):
            for card in self._cards:
                card.create_card()
