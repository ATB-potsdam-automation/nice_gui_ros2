from geopandas import GeoDataFrame
from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler
from nicegui import ui
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class SceneCard(Card):

    def __init__(self, node: Node, param_prefix: str) -> None:
        super().__init__(node, param_prefix)
        self.lat = self.lon = None  # GPS coordinates
        self._client_handler.add_subscriber(
            SubscriberHandler(
                node, NavSatFix, "/rover/gps_node/fix", self.update_gps_position
            )
        )

    def create_card(self) -> None:
        super().create_card()
        with ui.card().classes("w-300 h-300 text-center items-center"):
            options = {}  # See https://leafletjs.com/reference.html#map-option
            self.m = ui.leaflet(center=(51.505, -0.09), options=options).classes(
                "w-96 h-96"
            )
            ui.label().bind_text_from(
                self.m,
                "center",
                lambda center: f"Center: {center[0]:.3f}, {center[1]:.3f}",
            )
            ui.label().bind_text_from(self.m, "zoom", lambda zoom: f"Zoom: {zoom}")
            ui.button(
                "Center on robot position",
                on_click=lambda: self.center_on_robot(),
            )
            if self.lat and self.lon:
                self.marker = self.m.marker(latlng=(self.lat, self.lon))

            ui.timer(interval=1.0, callback=lambda: self.update_position())

    def update_gps_position(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def update_position(self) -> None:
        if self.lat and self.lon:
            self.marker.move(self.lat, self.lon)

    def center_on_robot(self) -> None:
        if self.lat and self.lon:
            self.m.center = (self.lat, self.lon)
            self.m.zoom = 18
        else:
            msg = "No GPS data available"
            self._logger.warn(msg)

    def draw_geo_data_frame(self, data: GeoDataFrame) -> None:
        # Create a geojson object
        geojson_data = data.__geo_interface__
        # Display the data on the map
        self.m.center = (data.total_bounds[1], data.total_bounds[0])
        self.m.generic_layer(name="geoJSON", args=[geojson_data])
        self.m.zoom = 16
