from nice_gui_core.cards import Card
from nice_gui_core.utils import get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter


class LinkCard(Card):
    def __init__(self, node: Node, param_prefix: str):
        super().__init__(node, param_prefix)
        self._links = get_parameter(
            node, param_prefix + ".links", Parameter.Type.STRING_ARRAY
        ).string_array_value

        self._link_urls = []
        self._link_names = []
        for link in self._links:
            link_url = get_parameter(
                node, link + ".url", Parameter.Type.STRING
            ).string_value
            link_name = get_parameter(
                node, link + ".name", Parameter.Type.STRING
            ).string_value
            self._link_urls.append(link_url)
            self._link_names.append(link_name)

    def create_card(self):
        super().create_card()
        with ui.card().classes("w-44 items-center"):
            ui.label(self._name).classes("text-2xl")
            for link_name, url in zip(self._link_names, self._link_urls):
                self.navigate(link_name, url)

    def navigate(self, link_name, url):
        return ui.button(
            link_name,
            on_click=lambda: ui.navigate.to(url),
        ).classes("mt-6")
