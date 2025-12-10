import os
import threading
from importlib import resources

import rclpy
from nice_gui_core.pages import PageWithCards
from nice_gui_core.utils import get_parameter
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


class NiceGuiNode(Node):
    """ROS2 node for NiceGUI"""

    def __init__(self) -> None:
        super().__init__("nicegui")

        # Create pages
        page_names = get_parameter(
            self, "pages", Parameter.Type.STRING_ARRAY
        ).string_array_value

        self._pages = []
        for page_name in page_names:
            self._pages.append(PageWithCards(self, page_name))


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but
    # it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    try:
        rclpy.init()
        executor = MultiThreadedExecutor()
        node = NiceGuiNode()
        executor.add_node(node)
        executor.spin()
    except ExternalShutdownException:
        # This exception is raised when the node is shut down externally
        # (e.g., by the user or by a signal)
        pass
    except BaseException:
        # This exception is raised when the node crashes or is shut down
        # unexpectedly
        app.shutdown()
        raise RuntimeError("ROS2 node has crashed or was shut down unexpectedly.")


app.on_startup(lambda: threading.Thread(target=ros_main).start())

# ROS2 uses a non-standard module name, so we need to specify it here
ui_run.APP_IMPORT_STRING = f"{__name__}:app"
with resources.path("nice_gui_core", "static") as static_dir:
    app.add_static_files("/static", static_dir)

ui.run(
    port=int(os.environ.get("NICEGUI_PORT", "8080")),
    dark=False,
    title=os.environ.get("NICEGUI_TITLE", "NICE_GUI"),
    reload=os.environ.get("NICEGUI_RELOAD", "True").lower() in ("1", "true", "yes"),
)
