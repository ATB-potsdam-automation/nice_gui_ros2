import base64

import cv2
import numpy as np
from nice_gui_core.cards import Card
from nice_gui_core.utils import SubscriberHandler, get_parameter
from nicegui import ui
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image


class ImageCard(Card):
    """A NiceGUI card that subscribes to a ROS2 image topic and displays it interactively."""

    def __init__(self, node: Node, param_prefix: str):
        """Initialize the Image Card
        Args:
            node (Node): the ROS2 node
            param_prefix (str): the parameter prefix for this card
        """
        super().__init__(node, param_prefix)

        topic = get_parameter(
            node, f"{param_prefix}.topic", Parameter.Type.STRING
        ).string_value
        compressed = get_parameter(
            node, f"{param_prefix}.compressed", Parameter.Type.BOOL
        ).bool_value
        self._scale = get_parameter(
            node, f"{param_prefix}.scale", Parameter.Type.INTEGER
        ).integer_value
        self._frequency = get_parameter(
            node, f"{param_prefix}.frequency", Parameter.Type.INTEGER
        ).integer_value

        self._image_base64 = None
        self._last_stamp = None
        self.video_image = None

        # Subscribe to ROS topic
        if compressed:
            self._client_handler.add_subscriber(
                SubscriberHandler(
                    node, CompressedImage, topic, self._image_compressed_callback
                )
            )
        else:
            self._client_handler.add_subscriber(
                SubscriberHandler(node, Image, topic, self._image_callback)
            )

    def _image_compressed_callback(self, msg: CompressedImage) -> None:
        """Handle compressed image messages."""
        if msg.header.stamp == self._last_stamp:
            return
        self._last_stamp = msg.header.stamp

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is not None:
            self._update_image(img)

    def _image_callback(self, msg: Image) -> None:
        """Handle uncompressed image messages.
        Args:
            msg (Image): the latest image message
        """
        if msg.header.stamp == self._last_stamp:
            return
        self._last_stamp = msg.header.stamp

        dtype = np.uint8
        img = None

        if msg.encoding == "bgr8":
            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                (msg.height, msg.width, 3)
            )
        elif msg.encoding == "rgb8":
            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                (msg.height, msg.width, 3)
            )
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "mono8":
            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                (msg.height, msg.width, 1)
            )
        else:
            raise NotImplementedError(f"Encoding '{msg.encoding}' not supported")

        if img is not None:
            self._update_image(img)

    def _update_image(self, img: np.ndarray):
        """Convert image to base64 and trigger UI update.
        Args:
            img (np.ndarray): the image array
        """
        if self._scale > 1:
            img = img[
                :: self._scale, :: self._scale
            ]  # Fast downscale by integer factor
        success, buffer = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            return

        image_base64 = base64.b64encode(buffer).decode("utf-8")
        self._image_base64 = image_base64

    def create_card(self):
        """Create the UI card with live image updates."""
        super().create_card()
        with ui.card().classes("w-96 h-110 items-center"):
            ui.label(self._name).classes("text-2xl")
            self.video_image = ui.interactive_image()
            ui.timer(
                1 / float(self._frequency), self._refresh_image
            )  # Refresh at the specified frequency

    def _refresh_image(self):
        """Update image in the UI only when new data is available."""
        if self._image_base64:
            self.video_image.set_source(f"data:image/jpeg;base64,{self._image_base64}")
            self.video_image.set_source(f"data:image/jpeg;base64,{self._image_base64}")
