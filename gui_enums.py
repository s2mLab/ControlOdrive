from enum import Enum
from utils import StrEnum


class TrainingMode(StrEnum):
    """
    The training modes implemented.
    """

    CONCENTRIC = "Concentric"
    ECCENTRIC = "Eccentric"


class GUIControlMode(StrEnum):
    """
    The control modes implemented.
    """

    LINEAR = "Linear"
    VELOCITY = "Velocity"
    TORQUE = "Torque"
    POWER = "Power"


class StopwatchStates(Enum):
    """
    The states of the stopwatch.
    """

    STOPPED = "Stopped"
    RUNNING = "Running"
    PAUSED = "Paused"
