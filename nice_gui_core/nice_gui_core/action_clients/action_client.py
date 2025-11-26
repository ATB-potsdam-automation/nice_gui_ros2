from typing import Any, Callable, Type

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nice_gui_core.logger import Logger
from rclpy.action import ActionClient as RosActionClient
from rclpy.action.client import ClientGoalHandle, Future
from rclpy.node import Node


class ActionClient:
    """Base class for NiceGUI action clients

    This class provides a simple interface for sending goals to ROS2 action servers.
    It gives a certain structure to the action clients used in NiceGUI. It is callback based,
    so the user does not need to wait for the action to finish and the ui stays responsive.

    Args:
        node (Node): the ROS2 node
        logger (Logger): the logger instance
        action_type (Type): the action type, e.g. FollowJointTrajectory
        action_name (str): the action name, e.g. joint_trajectory_controller/follow_joint_trajectory
    """

    def __init__(
        self,
        node: Node,
        logger: Logger,
        action_type: Type,
        action_name: str,
    ):
        """Initialize the action client class
        Args:
            node (Node): the ROS2 node
            logger (Logger): the logger instance
            action_type (Type): the action type, e.g. FollowJointTrajectory
            action_name (str): the action name, e.g. joint_trajectory_controller/follow_joint_trajectory
        """
        self._logger = logger.get_logger(action_name)
        self._node = node
        self._action_running = False
        self._action_client = RosActionClient(node, action_type, action_name)
        self._log_reduce = False

    def reduce_logging(self, reduce: bool) -> None:
        """Reduce logging output
        Args:
            reduce (bool): if true, reduce logging output
        """
        self._log_reduce = reduce

    def log_info(self, msg: str) -> None:
        """Log an info message if logging is not reduced
        Args:
            msg (str): the message to log
        """
        if not self._log_reduce:
            self._logger.info(msg)

    def log_positive(self, msg: str) -> None:
        """Log a positive message if logging is not reduced
        Args:
            msg (str): the message to log
        """
        if not self._log_reduce:
            self._logger.positive(msg)

    def log_warn(self, msg: str) -> None:
        """Log a warning message if logging is not reduced
        Args:
            msg (str): the message to log
        """
        if not self._log_reduce:
            self._logger.warn(msg)

    def send_goal(self, goal_msg) -> None:
        """Send the commmand
        Args:
            goal_msg: the goal message
        """
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self._logger.error(f"Action server not available, try again later...")
            self._on_aborted()

        self._goal_handle = None
        self._feedback = None

        self._logger.debug(f"Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def cancel_goal(self):
        """Cancel the current goal"""
        if not self._action_running:
            self.log_warn("No action running, nothing to cancel.")
            return

        if self._goal_handle is None:
            self._logger.error("No goal handle, cannot cancel.")
            self._on_aborted()
            return

        self._logger.debug("Cancelling the current goal...")
        cancel_future: Future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def set_on_goal_rejected(self, on_goal_rejected: Callable) -> None:
        """Set the callback for goal rejected
        Args:
            on_goal_rejected (Callable): the callback function
        """
        self._on_goal_rejected = on_goal_rejected

    def set_on_goal_accepted(self, on_goal_accepted: Callable) -> None:
        """Set the callback for goal accepted
        Args:
            on_goal_accepted (Callable): the callback function
        """
        self._on_goal_accepted = on_goal_accepted

    def set_on_feedback(self, on_feedback: Callable) -> None:
        """Set the callback for feedback message
        Args:
            on_feedback (Callable): the callback function
        """
        self._on_feedback = on_feedback

    def set_on_succeeded(self, on_succeeded: Callable) -> None:
        """Set the callback for succeeded
        Args:
            on_succeeded (Callable): the callback function
        """
        self._on_succeeded = on_succeeded

    def set_on_canceled(self, on_canceled: Callable) -> None:
        """Set the callback for canceled
        Args:
            on_canceled (Callable): the callback function
        """
        self._on_canceled = on_canceled

    def set_on_aborted(self, on_aborted: Callable) -> None:
        """Set the callback for aborted
        Args:
            on_aborted (Callable): the callback function
        """
        self._on_aborted = on_aborted

    def is_running(self) -> bool:
        """Return true if following action machine is running
        Returns:
            bool: true if action is running
        """
        return self._action_running

    def _goal_response_callback(self, future: Future):
        """Callback on goal request
        Args:
            future: the future object
        """
        self._goal_handle: ClientGoalHandle = future.result()
        if self._goal_handle is None:
            self._logger.error("Goal handle is none")
            return

        status: GoalStatus = self._goal_handle.status
        if self._goal_handle.accepted:
            self.log_positive("Goal accepted")
            result_future: Future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
            self._action_running = True
            self._on_goal_accepted()
        else:
            self._logger.error(f"Goal rejected with status: {status}")
            self._action_running = False
            self._on_goal_rejected()

    def _result_callback(self, future: Future):
        """Callback on result message
        Args:
            future: the future object
        """
        self._action_running = False
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log_positive("Action succeeded")
            self._on_succeeded(future.result().result)
        elif status == GoalStatus.STATUS_ABORTED:
            self._logger.error("Action aborted")
            self._on_aborted()
        elif status == GoalStatus.STATUS_CANCELED:
            self.log_warn("Action canceled")
            self._on_canceled()
        else:
            self._logger.error(f"Action failed with status: {status}")
            self._on_aborted()

    def _feedback_callback(self, feedback_msg: Any):
        """Callback on feedback message
        Args:
            feedback_msg: the feedback message
        """
        self._logger.debug(f"Feedback: {feedback_msg.feedback}")
        self._on_feedback(feedback_msg.feedback)

    def _cancel_done_callback(self, future: Future):
        """Callback on cancel request
        Args:
            future: the future object
        """
        cancel_response: CancelGoal.Response = future.result()
        if cancel_response.return_code == CancelGoal.Response.ERROR_NONE:
            self._logger.debug("Cancel request accepted")
        else:
            self._logger.error(
                f"Goal cancel failed with: {cancel_response.return_code}"
            )

    def _on_goal_rejected(self):
        """Callback on goal rejected"""
        pass

    def _on_goal_accepted(self):
        """Callback on goal accepted"""
        pass

    def _on_feedback(self, feedback_msg: Any):
        """Callback on feedback message
        Args:
            feedback_msg: the feedback message
        """
        pass

    def _on_succeeded(self, result_msg: Any):
        """Callback on result message
        Args:
            result_msg: the result message
        """
        pass

    def _on_canceled(self):
        """Callback on goal canceled"""
        pass

    def _on_aborted(self):
        """Callback on goal aborted"""
        pass
