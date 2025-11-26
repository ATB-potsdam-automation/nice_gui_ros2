from typing import Callable

from nicegui import ui


class BtnActionStartStop:
    """Button for starting and stopping an action
    The internal state allowed to remember state on page refresh.
    """

    def __init__(self) -> None:
        """Initialize the button"""
        self._started = False

    def create_ui_element(self) -> None:
        """Create the UI element"""
        with ui.row().classes("w-full justify-center"):
            self._btn_start = ui.button(
                "Start",
                on_click=self._on_start_click,
                icon="play_arrow",
            )
            self._btn_stop = ui.button(
                "Stop",
                on_click=self._on_stop_click,
                icon="stop",
            )
        self._set_btn()

    def set_color(self) -> None:
        self._btn_start.props("color=primary")
        self._btn_stop.props("color=negative")

    def set_on_start_callback(self, callback: Callable) -> None:
        """Set the callback for the start button"""
        self._on_start_callback = callback

    def set_on_stop_callback(self, callback: Callable) -> None:
        """Set the callback for the stop button"""
        self._on_stop_callback = callback

    def _on_start_click(self) -> None:
        """Handle the start button click"""
        self.start()
        self._on_start_callback()

    def _on_stop_click(self) -> None:
        """Handle the stop button click"""
        self.stop()
        self._on_stop_callback()

    def start(self) -> None:
        """Set the button to started state"""
        self._started = True
        self._set_btn()

    def stop(self) -> None:
        """Set the button to stopped state"""
        self._started = False
        self._set_btn()

    def _set_btn(self) -> None:
        """Set the button state"""
        if self._started:
            self._btn_start.disable()
            self._btn_stop.enable()
        else:
            self._btn_start.enable()
            self._btn_stop.disable()
        self.set_color()

    def _on_start_callback(self) -> None:
        """Callback for the start button. Do nothing by default."""
        pass

    def _on_stop_callback(self) -> None:
        """Callback for the stop button. Do nothing by default."""
        pass
