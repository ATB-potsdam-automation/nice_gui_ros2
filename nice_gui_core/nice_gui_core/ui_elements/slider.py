from nice_gui_core.utils import get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter


class Slider:
    """A slider UI element that gets its configuration from ROS2 parameters."""

    def __init__(self, node: Node, name: str, param_prefix: str) -> None:
        """Initialize the slider with parameters from the ROS2 node.
        Args:
            node (Node): The ROS2 node to get parameters from.
            name (str): The name of the slider parameter set.
            param_prefix (str): The prefix for the parameter names.
        """
        self.name = name
        self.value = get_parameter(
            node, f"{param_prefix}.{name}.init", Parameter.Type.DOUBLE
        ).double_value
        self._min = get_parameter(
            node, f"{param_prefix}.{name}.min", Parameter.Type.DOUBLE
        ).double_value
        self._max = get_parameter(
            node, f"{param_prefix}.{name}.max", Parameter.Type.DOUBLE
        ).double_value
        self._step_size = get_parameter(
            node, f"{param_prefix}.{name}.step_size", Parameter.Type.DOUBLE
        ).double_value

    def create_ui_element(self):
        """Create the slider UI element."""
        with ui.grid(columns=2):
            ui.label(f"{self.name}:").classes("text-left")
            self.label_value = ui.label(f"{self.value:.2f}").classes("text-right")
        slider = ui.slider(
            value=self.value,
            min=self._min,
            max=self._max,
            step=self._step_size,
            on_change=lambda e: self.set_value(e.value),
        )
        self.label_value.bind_text_from(slider, "value")

    def set_value(self, value: float):
        """Set the current value of the slider.
        Args:
            value (float): The new value to set.
        """
        self.value = value

    def get_value(self) -> float:
        """Get the current value of the slider.
        Returns:
            float: The current value of the slider.
        """
        return self.value
