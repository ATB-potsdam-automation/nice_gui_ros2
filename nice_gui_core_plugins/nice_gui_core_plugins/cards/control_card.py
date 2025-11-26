from geometry_msgs.msg import Twist
from nice_gui_core.cards import Card
from nice_gui_core.utils import get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter


class ControlCard(Card):
    """
    Create card to control robot speed.

    With this card we can control the speed in x-direction and the yaw rate of a robot.
    """

    def __init__(self, node: Node, param_prefix: str) -> None:
        """
        Construct all the nexessary attributes for the control card object.

        Parameters
        -----------
        node: Node
            ROS2 node

        Returns
        -----------
        None
        """

        super().__init__(node, param_prefix)
        self.cmd_vel_topic = get_parameter(
            node, param_prefix + ".cmd_vel_topic", Parameter.Type.STRING
        ).string_value
        self.cmd_vel_publisher = node.create_publisher(Twist, self.cmd_vel_topic, 1)
        self._max_speed_linear = get_parameter(
            node, param_prefix + ".max_speed.linear", Parameter.Type.DOUBLE
        ).double_value
        self._max_speed_angular = get_parameter(
            node, param_prefix + ".max_speed.angular", Parameter.Type.DOUBLE
        ).double_value
        self._max_speed_linear_turbo = get_parameter(
            node, param_prefix + ".max_speed.linear_turbo", Parameter.Type.DOUBLE
        ).double_value
        self._max_speed_angular_turbo = get_parameter(
            node, param_prefix + ".max_speed.angular_turbo", Parameter.Type.DOUBLE
        ).double_value
        self._current_max_speed_linear = self._max_speed_linear
        self._current_max_speed_angular = self._max_speed_angular
        self._control_activated = False

    def create_card(self) -> None:
        """
        Create the nicegui card object.

        Parameters
        -----------
        None

        Returns
        -----------
        None
        """
        super().create_card()
        self.switch_turbo_mode(False)
        self.switch_activate(False)
        with ui.card().classes("w-44 text-center items-center"):
            ui.label(self._name).classes("text-2xl")
            ui.switch(
                "Activate", on_change=lambda e: self.switch_activate(e.value)
            ).classes("mt-6")
            ui.joystick(
                color="blue",
                size=50,
                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                on_end=lambda _: self.send_speed(0.0, 0.0),
            )
            ui.switch(
                "Turbo mode", on_change=lambda e: self.switch_turbo_mode(e.value)
            ).classes("mt-6")
            ui.label(
                "Publish steering commands by dragging your mouse around in the blue field"
            ).classes("mt-6")

    def switch_activate(self, activate: bool) -> None:
        """
        Activate Control

        Parameters
        -----------
        activate: bool
            True if we want to activate control

        Returns
        -----------
        None
        """
        self._control_activated = activate

    def switch_turbo_mode(self, turbo: bool) -> None:
        """
        Switch between turbo and normal mode.

        Parameters
        -----------
        turbo: bool
            True if turbo mode is active

        Returns
        -----------
        None
        """
        self._logger.debug(f"Turbo mode: {turbo}")
        if turbo:
            self._current_max_speed_linear = self._max_speed_linear_turbo
            self._current_max_speed_angular = self._max_speed_angular_turbo
        else:
            self._current_max_speed_linear = self._max_speed_linear
            self._current_max_speed_angular = self._max_speed_angular

    def send_speed(self, x: float, yaw: float) -> None:
        """
        Publish the speed command on the ros2 publisher.

        Parameters
        ------------
        x: float
            speed in x or forward direction
        yaw: float
            yaw-rate of the speed

        Returns
        ----------
        None
        """
        if not self._control_activated:
            return

        cmd_message = Twist()
        cmd_message.linear.x, cmd_message.linear.y, cmd_message.linear.z = (
            self._current_max_speed_linear * float(x),
            0.0,
            0.0,
        )
        cmd_message.angular.x, cmd_message.angular.y, cmd_message.angular.z = (
            0.0,
            0.0,
            self._current_max_speed_angular * float(-yaw),
        )
        self.cmd_vel_publisher.publish(cmd_message)
