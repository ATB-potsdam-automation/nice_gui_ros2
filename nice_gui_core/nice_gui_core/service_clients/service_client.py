from typing import Any, Callable, Type

from nice_gui_core.logger import Logger
from rclpy.node import Node


class ServiceClient:
    """Base class for NiceGUI service clients

    This class provides a simple interface for calling ROS2 services.
    It gives a certain structure to the service clients used in NiceGUI. It is callback based,
    so the user does not need to wait for the service to finish and the ui stays responsive.

    Args:
        node (Node): the ROS2 node
        logger (Logger): the logger instance
        *args: arguments for the service client
        **kwargs: keyword arguments for the service client
    """

    def __init__(
        self,
        node: Node,
        logger: Logger,
        service_type: Type,
        service_name: str,
        *args,
        **kwargs,
    ):
        """Initialize the service client class"""
        super().__init__(*args, **kwargs)
        self._logger = logger.get_logger(service_name)
        self._node = node
        self._service_running = False
        self._client = node.create_client(srv_type=service_type, srv_name=service_name)

    def is_running(self) -> bool:
        """Return true if service call is running
        Returns:
            bool: true if service call is running
        """
        return self._service_running

    def call(self, request: Any) -> None:
        """Call the service
        Args:
            request: the service request
        """
        if not self._client.wait_for_service(timeout_sec=1.0):
            self._logger.error(f"Service not available, try again later...")
            self._on_aborted()

        self._logger.debug(f"Calling service...")
        self._service_running = True
        self._call_future = self._client.call_async(request)
        self._call_future.add_done_callback(self._call_done_callback)

    def set_on_succeeded(self, callback: Callable) -> None:
        """Set the callback function when the service call succeeded
        Args:
            callback (Callable): the callback function
        """
        self._on_succeeded = callback

    def set_on_aborted(self, callback: Callable) -> None:
        """Set the callback function when the service call aborted
        Args:
            callback (Callable): the callback function
        """
        self._on_aborted = callback

    def _call_done_callback(self, future) -> None:
        """Callback on service call
        Args:
            future: the future object
        """
        response = future.result()
        if response is None:
            self._logger.error(f"Service call failed")
            self._on_aborted()
        else:
            self._logger.debug(f"{response}")
            self._on_succeeded()
        self._service_running = False

    def _on_succeeded(self) -> None:
        """Callback when the service call succeeded"""
        pass

    def _on_aborted(self) -> None:
        """Callback when the service call aborted"""
        pass
