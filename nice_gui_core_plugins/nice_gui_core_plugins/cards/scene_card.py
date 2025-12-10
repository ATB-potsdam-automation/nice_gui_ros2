from geopandas import GeoDataFrame
from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler, get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix


class SceneCard(Card):
    """Class for Scene Card displaying GPS data on a map"""

    def __init__(self, node: Node, param_prefix: str) -> None:
        """Initialize the Scene Card
        Args:
            node (Node): the ROS2 node
            param_prefix (str): the parameter prefix for this card
        """
        super().__init__(node, param_prefix)
        self.lat = self.lon = None  # GPS coordinates
        self._gps_topic: str = get_parameter(
            node, param_prefix + ".gps_topic", Parameter.Type.STRING
        ).string_value
        self._client_handler.add_subscriber(
            SubscriberHandler(
                node, NavSatFix, self._gps_topic, self.update_gps_position
            )
        )

    def create_card(self) -> None:
        """Create the Scene Card layout"""
        super().create_card()
        with ui.card().classes("max-w-300 max-h-300 text-center items-center"):
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
        """Update GPS position from NavSatFix message
        Args:
            msg (NavSatFix): The latest GPS message
        """
        self.lat = msg.latitude
        self.lon = msg.longitude

    def update_position(self) -> None:
        """Update the marker position on the map"""
        if self.lat and self.lon:
            self.marker.move(self.lat, self.lon)

    def center_on_robot(self) -> None:
        """Center the map on the robot's current GPS position"""
        if self.lat and self.lon:
            self.m.center = (self.lat, self.lon)
            self.m.zoom = 18
        else:
            self._logger.warn(f"No GPS data available. Check topic {self._gps_topic}")

    def draw_geo_data_frame(self, data: GeoDataFrame) -> None:
        """Draw GeoDataFrame data on the map
        Args:
            data (GeoDataFrame): The geospatial data to display
        """
        geojson_data = data.__geo_interface__
        self.m.center = (data.total_bounds[1], data.total_bounds[0])
        self.m.generic_layer(name="geoJSON", args=[geojson_data])
        self.m.zoom = 16
        self.m.zoom = 16
        self.m.zoom = 16
        self.m.zoom = 16
        self.m.zoom = 16
