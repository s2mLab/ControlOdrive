from enum import Enum

class StrEnum(str, Enum):
    """
    Enum where members are also (and must be) strings. Existing class in Python 3.11 but the code is running in Python
    3.9 for now.
    """

    def __new__(cls, *values):
        "values must already be of type `str`"
        if len(values) > 3:
            raise TypeError('too many arguments for str(): %r' % (values, ))
        if len(values) == 1:
            # it must be a string
            if not isinstance(values[0], str):
                raise TypeError('%r is not a string' % (values[0], ))
        if len(values) >= 2:
            # check that encoding argument is a string
            if not isinstance(values[1], str):
                raise TypeError('encoding must be a string, not %r' % (values[1], ))
        if len(values) == 3:
            # check that errors argument is a string
            if not isinstance(values[2], str):
                raise TypeError('errors must be a string, not %r' % (values[2]))
        value = str(*values)
        member = str.__new__(cls, value)
        member._value_ = value
        return member

    def _generate_next_value_(name, start, count, last_values):
        """
        Return the lower-cased version of the member name.
        """
        return name.lower()


class ControlMode(StrEnum):
    """
    The control modes implemented.
    """

    STOP = "STOP"
    STOPPING = "Stopping"
    POSITION_CONTROL = "Position control"
    LINEAR_CONTROL = "Linear control"
    VELOCITY_CONTROL = "Velocity control"
    TORQUE_CONTROL = "Torque control"
    POWER_CONTROL = "Power control"
    TEST = "Test"
    ECC_VEL_SECU = "Eccentric velocity security"


control_modes_based_on_torque = ControlMode.TORQUE_CONTROL + ControlMode.POWER_CONTROL + ControlMode.LINEAR_CONTROL


class TrainingMode(StrEnum):
    """
    The training modes implemented.
    """

    ECCENTRIC = "Eccentric"
    CONCENTRIC = "Concentric"
