from math import atan2
from typing import List

from nav_msgs.msg import Odometry, Path
from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler, get_parameter
from nice_gui_core_plugins.utils import NiceguiPath
from nicegui import ui
from nicegui.elements.scene import Scene
from rclpy.node import Node
from rclpy.parameter import Parameter


class GridVisualizationCard(Card):
    """Class for Grid Visualization Card"""

    def __init__(self, node: Node, param_prefix: str):
        """Initialize the Grid Visualization Card
        Args:
            node (Node): the ROS2 node
            param_prefix (str): the parameter prefix for this card
        """
        super().__init__(node, param_prefix)
        self.robot_3d = None
        self._scene = None
        self._path: Scene | None = None
        self._odom: Odometry | None = None

        odom_topic: str = get_parameter(
            node, param_prefix + ".odom_topic", Parameter.Type.STRING
        ).string_value
        self._client_handler.add_subscriber(
            SubscriberHandler(node, Odometry, odom_topic, self.odom_callback)
        )

        path_subscriber_topics: List[str] = get_parameter(
            node, param_prefix + ".path_subscriber_topics", Parameter.Type.STRING_ARRAY
        ).string_array_value
        for topic in path_subscriber_topics:
            self._client_handler.add_subscriber(
                SubscriberHandler(node, Path, topic, self.path_callback)
            )

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function for odometry data
        Args:
            msg (Odometry): the latest odometry data
        """
        self._odom = msg
        self.handle_pose(msg)

    def path_callback(self, msg: Path) -> None:
        """Callback function for path data
        Args:
            msg (Path): the latest path data
        """
        self.draw_path(msg)

    def create_card(self):
        """Create the Grid Visualization Card layout"""
        super().create_card()
        with ui.card().classes("w-96 h-110 items-center"):
            ui.label(self._name).classes("text-2xl")
            self._scene = ui.scene(
                width=350, height=300, camera=ui.scene.perspective_camera()
            )
            with self._scene.group() as self.robot_3d:
                prism = [
                    [-0.5, -0.5],
                    [0.5, -0.5],
                    [0.75, 0],
                    [0.5, 0.5],
                    [-0.5, 0.5],
                ]
                self.robot_object = self._scene.extrusion(prism, 0.4).material(
                    "#4488ff", 0.5
                )
            self._path = NiceguiPath(self._scene)
            ui.button(
                "Center on robot position",
                on_click=lambda: self.center_on_robot(),
            )

    def draw_path(self, path: Path | None):
        """Draw the path in the 3D scene
        Args:
            path (Path | None): the path to draw, or None to delete the path
        """
        if path is None:
            self._path.delete()
        else:
            self._path.draw(path)

    def handle_pose(self, msg: Odometry) -> None:
        """Handle the robot pose update
        Args:
            msg (Odometry): the latest odometry data
        """
        if self.robot_3d:
            self.robot_3d.move(msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.robot_3d.rotate(
                0,
                0,
                2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            )

    def center_on_robot(self) -> None:
        """Center the camera on the robot position"""
        if self._odom is None:
            self._logger.warn("No robot position available")
            return

        # I needed to set the camera up_x, up_y, up_z to None before moving the camera.
        # Otherwise the look_at_x and look_at_y always return to zero. I guess there is the bug
        # either in nicegui or in the threejs library.
        self._scene.camera.up_x = None
        self._scene.camera.up_y = None
        self._scene.camera.up_z = None
        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        # Subtract a small number from the y component, otherwise the camera rotation vector seems not to be clear
        # and the viewing angle on the robot rotates by 90 degree from time to time.
        self._scene.move_camera(x=x, y=y - 0.01, look_at_x=x, look_at_y=y, duration=0)
        self._scene.move_camera(x=x, y=y - 0.01, look_at_x=x, look_at_y=y, duration=0)
        self._scene.move_camera(x=x, y=y - 0.01, look_at_x=x, look_at_y=y, duration=0)
