import math, signal
from typing import Optional,Callable, Literal, Type, Tuple
from types import FrameType
from synchros2.action_handle import ActionHandle
import logging
from synchros2.utilities import fqn
from bosdyn_msgs.conversions import convert

from synchros2.action_client import ActionClientWrapper
from synchros2.type_hints import Action



class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""

    def __init__(self) -> None:
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self) -> "ExitCheck":
        return self

    def __exit__(
        self,
        _type: Optional[Type[BaseException]],
        _value: Optional[BaseException],
        _traceback: Optional[FrameType],
    ) -> Literal[False]:
        return False

    def _sigterm_handler(self, _signum: int, _frame: Optional[FrameType]) -> None:
        self._kill_now = True

    def request_exit(self) -> None:
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self) -> bool:
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now



class ActionMockProxy:
    """This wrapper for the ActionClientWrapper skips sending actions when the mock is True.
    When using the mock driver, sending actions to it crashes it, so for testing the code with mock.
    """
    def __init__(self,original:ActionClientWrapper,mock:bool):
        self._logger = logging.getLogger(fqn(self.__class__))
        self._original = original
        self._mock = mock
                
    def send_goal_and_wait(self, action_name: str,
        goal: Action.Goal,
        timeout_sec: Optional[float] = None,
    ) -> Optional[Action.Result]:
        if self._mock:
            self._logger.info("Mock enabled, passing function")
            return None
        else:
            return self._original.send_goal_and_wait(action_name, goal, timeout_sec)
    
    def send_goal_async_handle(
        self,
        action_name: str,
        goal: Action.Goal,
        *,
        result_callback: Optional[Callable[[Action.Result], None]] = None,
        feedback_callback: Optional[Callable[[Action.Feedback], None]] = None,
        on_failure_callback: Optional[Callable[[], None]] = None,
    ) -> ActionHandle:
        if self._mock:
            self._logger.info("Mock enabled, passing function")
            return ActionHandle(action_name)
        else:
            self._logger.info("Mock not, passing function")
            return self._original.send_goal_async_handle(action_name,goal,
                                                  result_callback=result_callback,
                                                  feedback_callback=feedback_callback,
                                                  on_failure_callback=on_failure_callback)

def get_enum_name(msg_type, value, group: Optional[str] = None) -> str:
    """Find the constant name for a given value in a ROS 2 message type
    Args:
        msg_type: The ROS 2 message class type
        value: The value to find
        group: Optional group to filter the attribute names
    Returns:
        The name of the constant with the given value
    """
    for attr, val in vars(msg_type).items():
        if isinstance(val, int) and val == value:
            # If a group is specified, check if the group is part of the attribute name
            if group is None or group in attr:
                return attr
    return f"UNKNOWN ({value})"


COLORS = {
        "black": "\033[30m",
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[37m",
        "reset": "\033[0m",
    }

def color_text(text: str, color: str) -> str:
    return f"{COLORS.get(color, COLORS['reset'])}{text}{COLORS['reset']}"