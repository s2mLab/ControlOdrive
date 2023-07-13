"""
Utility functions.
"""
from enum import Enum


class StrEnum(str, Enum):
    """
    Enum where members are also (and must be) strings. Existing class in Python 3.11 but the code is running in Python
    3.9 for now.
    """

    def __new__(cls, *values):
        """
        Values must already be of type `str`.
        """
        if len(values) > 3:
            raise TypeError("too many arguments for str(): %r" % (values,))
        if len(values) == 1:
            # it must be a string
            if not isinstance(values[0], str):
                raise TypeError("%r is not a string" % (values[0],))
        if len(values) >= 2:
            # check that encoding argument is a string
            if not isinstance(values[1], str):
                raise TypeError("encoding must be a string, not %r" % (values[1],))
        if len(values) == 3:
            # check that errors argument is a string
            if not isinstance(values[2], str):
                raise TypeError("errors must be a string, not %r" % (values[2]))
        value = str(*values)
        member = str.__new__(cls, value)
        member._value_ = value
        return member

    def _generate_next_value_(self, start, count, last_values):
        """
        Return the lower-cased version of the member name.
        """
        return self.lower()


class ODriveError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    ODRIVE_ERROR_NONE = 0
    ODRIVE_ERROR_CONTROL_ITERATION_MISSED = 1
    ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE = 2
    ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE = 4
    ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT = 8
    ODRIVE_ERROR_DC_BUS_OVER_CURRENT = 16
    ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION = 32
    ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN = 64
    ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE = 128


class ODriveAxisError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    AXIS_ERROR_NONE = 0
    AXIS_ERROR_INVALID_STATE = 1
    AXIS_ERROR_MOTOR_FAILED = 64
    AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED = 128
    AXIS_ERROR_ENCODER_FAILED = 256
    AXIS_ERROR_CONTROLLER_FAILED = 512
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 2048
    AXIS_ERROR_MIN_ENDSTOP_PRESSED = 4096
    AXIS_ERROR_MAX_ENDSTOP_PRESSED = 8192
    AXIS_ERROR_ESTOP_REQUESTED = 16384
    AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 131072
    AXIS_ERROR_OVER_TEMP = 262144
    AXIS_ERROR_UNKNOWN_POSITION = 524288


class ODriveControllerError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    CONTROLLER_ERROR_NONE = 0
    CONTROLLER_ERROR_OVERSPEED = 1
    CONTROLLER_ERROR_INVALID_INPUT_MODE = 2
    CONTROLLER_ERROR_UNSTABLE_GAIN = 4
    CONTROLLER_ERROR_INVALID_MIRROR_AXIS = 8
    CONTROLLER_ERROR_INVALID_LOAD_ENCODER = 16
    CONTROLLER_ERROR_INVALID_ESTIMATE = 32
    CONTROLLER_ERROR_INVALID_CIRCULAR_RANGE = 64
    CONTROLLER_ERROR_SPINOUT_DETECTED = 128


class ODriveEncoderError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    ENCODER_ERROR_NONE = 0
    ENCODER_ERROR_UNSTABLE_GAIN = 1
    ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH = 2
    ENCODER_ERROR_NO_RESPONSE = 4
    ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE = 8
    ENCODER_ERROR_ILLEGAL_HALL_STATE = 16
    ENCODER_ERROR_INDEX_NOT_FOUND_YET = 32
    ENCODER_ERROR_ABS_SPI_TIMEOUT = 64
    ENCODER_ERROR_ABS_SPI_COM_FAIL = 128
    ENCODER_ERROR_ABS_SPI_NOT_READY = 256
    ENCODER_ERROR_HALL_NOT_CALIBRATED_YET = 512


class ODriveMotorError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    MOTOR_ERROR_NONE = 0
    MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 1
    MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 2
    MOTOR_ERROR_DRV_FAULT = 8
    MOTOR_ERROR_CONTROL_DEADLINE_MISSED = 16
    MOTOR_ERROR_MODULATION_MAGNITUDE = 128
    MOTOR_ERROR_CURRENT_SENSE_SATURATION = 1024
    MOTOR_ERROR_CURRENT_LIMIT_VIOLATION = 4096
    MOTOR_ERROR_MODULATION_IS_NAN = 65536
    MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP = 131072
    MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP = 262144
    MOTOR_ERROR_TIMER_UPDATE_MISSED = 524288
    MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 1048576
    MOTOR_ERROR_CONTROLLER_FAILED = 2097152
    MOTOR_ERROR_I_BUS_OUT_OF_RANGE = 4194304
    MOTOR_ERROR_BRAKE_RESISTOR_DISARMED = 8388608
    MOTOR_ERROR_SYSTEM_LEVEL = 16777216
    MOTOR_ERROR_BAD_TIMING = 33554432
    MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE = 67108864
    MOTOR_ERROR_UNKNOWN_PHASE_VEL = 134217728
    MOTOR_ERROR_UNKNOWN_TORQUE = 268435456
    MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND = 536870912
    MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT = 1073741824
    MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE = 2147483648
    MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND = 4294967296
    MOTOR_ERROR_UNKNOWN_GAINS = 8589934592
    MOTOR_ERROR_CONTROLLER_INITIALIZING = 17179869184
    MOTOR_ERROR_UNBALANCED_PHASES = 34359738368


class ODriveSensorlessEstimatorError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    SENSORLESS_ESTIMATOR_ERROR_NONE = 0
    SENSORLESS_ESTIMATOR_ERROR_UNSTABLE_GAIN = 1
    SENSORLESS_ESTIMATOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT = 2


class ODriveCanError(Enum):
    """
    See https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html? to have more information.
    """

    CAN_ERROR_NONE = 0
    CAN_ERROR_DUPLICATE_CAN_IDS = 1


def traduce_error(
    decimal_number: int,
    odrive_enum: Enum,
) -> str:
    """
    Traduce an error code into a string.

    Parameters
    ----------
    decimal_number : int
        Error code.
    odrive_enum : Enum
        The enum containing the error codes.

    Returns
    -------
    error_code: str
        The error code(s) in string format.
    """
    error_code = ""
    highest_inferior = 0
    error = ""
    for member in odrive_enum.__members__.values():
        if highest_inferior < member.value <= decimal_number:
            highest_inferior = member.value
            error = member.name
    if error != "":
        error_code += error + ", "
    if decimal_number - highest_inferior > 0:
        error_code += traduce_error(decimal_number - highest_inferior, odrive_enum)
    return error_code
