"""
This module contains the enums used in the motor control. The control modes, the direction modes and the ODrive errors.
"""
from ergocycleS2M.utils import StrEnum


class ControlMode(StrEnum):
    """
    The control modes implemented.
    """

    STOP = "STOP"
    STOPPING = "Stopping"
    POSITION_CONTROL = "Position control"
    LINEAR_CONTROL = "Linear control"
    CADENCE_CONTROL = "Cadence control"
    TORQUE_CONTROL = "Torque control"
    CONCENTRIC_POWER_CONTROL = "Concentric power control"
    ECCENTRIC_POWER_CONTROL = "Eccentric power control"


# Thanks to the StrEnum, we can use the following syntax:
control_modes_based_on_torque = (
    ControlMode.TORQUE_CONTROL + ControlMode.CONCENTRIC_POWER_CONTROL + ControlMode.LINEAR_CONTROL
)
control_modes_based_on_cadence = ControlMode.CADENCE_CONTROL + ControlMode.ECCENTRIC_POWER_CONTROL


class DirectionMode(StrEnum):
    """
    The training modes implemented.
    """

    REVERSE = "Reverse"
    FORWARD = "Forward"
