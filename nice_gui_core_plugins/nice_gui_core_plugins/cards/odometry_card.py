from datetime import datetime

from nav_msgs.msg import Odometry
from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler, get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter


class OdometryCard(Card):
    """Card for starting and stopping the weed removal process"""

    def __init__(self, node: Node, param_prefix: str) -> None:
        """Initialize the card
        Args:
            node (Node): the ROS2 node
            param_prefix (str): the parameter prefix for the card
        """
        super().__init__(node, param_prefix)
        self._min_vel_x = get_parameter(
            node, f"{param_prefix}.min_vel_x", Parameter.Type.DOUBLE
        ).double_value
        self._max_vel_x = get_parameter(
            node, f"{param_prefix}.max_vel_x", Parameter.Type.DOUBLE
        ).double_value
        self._min_plot_y = get_parameter(
            node, f"{param_prefix}.plot.min_y", Parameter.Type.DOUBLE
        ).double_value
        self._max_plot_y = get_parameter(
            node, f"{param_prefix}.plot.max_y", Parameter.Type.DOUBLE
        ).double_value
        self._plot_update_every = get_parameter(
            node, f"{param_prefix}.plot.update_every", Parameter.Type.INTEGER
        ).integer_value
        self._plot_limit_number = get_parameter(
            node, f"{param_prefix}.plot.limit_number", Parameter.Type.INTEGER
        ).integer_value
        topic = get_parameter(
            node, f"{param_prefix}.topic", Parameter.Type.STRING
        ).string_value
        self._odom_msg = Odometry()
        self._client_handler.add_subscriber(
            SubscriberHandler(node, Odometry, topic, self._odom_callback)
        )

    def create_card(self) -> None:
        """Create the card"""
        super().create_card()
        with ui.card().classes("w-84 text-center items-center h-96"):
            ui.label(self._name).classes("text-2xl")
            with ui.grid(rows=3).classes("w-full"):
                self._line_plot = ui.line_plot(
                    n=3,
                    limit=self._plot_limit_number,
                    # figsize=(6, 4),
                    update_every=self._plot_update_every,
                )
                self._line_plot.fig.axes[0].set_xticklabels([])
                self._line_plot.fig.axes[0].lines[1].set_linestyle("--")
                self._line_plot.fig.axes[0].lines[2].set_linestyle("--")
                self._line_plot.fig.axes[0].lines[1].set_color("black")
                self._line_plot.fig.axes[0].lines[2].set_color("black")
                self._line_plot.fig.axes[0].legend(
                    ["velocity", "min", "max"],
                    loc="upper center",
                    ncol=3,
                )

                ui.timer(0.1, self.update_line_plot, active=True)

    def _odom_callback(self, msg: Odometry) -> None:
        """Callback for odometry messages
        Args:
            msg (Odometry): the odometry message
        """
        self._odom_msg = msg

    def update_line_plot(self) -> None:
        """Update the line plot with new data"""
        now = datetime.now()
        x = round(now.timestamp(), 1)
        y1 = self._odom_msg.twist.twist.linear.x
        self._line_plot.push(
            [now],
            [[y1], [self._min_vel_x], [self._max_vel_x]],
            y_limits=(self._min_plot_y, self._max_plot_y),
        )
