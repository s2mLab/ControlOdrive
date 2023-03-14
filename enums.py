from enum import Enum


class ControlMode(Enum):
    """
    The control modes implemented.
    """

    STOP = "stop"
    POSITION_CONTROL = "position_control"
    VELOCITY_CONTROL = "velocity_control"
    TORQUE_CONTROL = "torque_control"
    POWER_CONTROL = "power_control"


class PowerMode(Enum):
    """
    The power control modes implemented.
    """

    CONSTANT = "constant"
    LINEAR = "linear"
