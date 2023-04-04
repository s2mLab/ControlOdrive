from enum import Enum


class StrEnum(str, Enum):
    """
    Enum where members are also (and must be) strings. Existing class in Python 3.11 but the code is running in Python
    3.9 for now.
    """

    def __new__(cls, *values):
        "values must already be of type `str`"
        if len(values) > 3:
            raise TypeError('too many arguments for str(): %r' % (values,))
        if len(values) == 1:
            # it must be a string
            if not isinstance(values[0], str):
                raise TypeError('%r is not a string' % (values[0],))
        if len(values) >= 2:
            # check that encoding argument is a string
            if not isinstance(values[1], str):
                raise TypeError('encoding must be a string, not %r' % (values[1],))
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


class ODriveError(StrEnum):
    ODRIVE_ERROR_NONE = "0x00000000"
    ODRIVE_ERROR_CONTROL_ITERATION_MISSED = "0x00000001"
    ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE = "0x00000002"
    ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE = "0x00000004"
    ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT = "0x00000008"
    ODRIVE_ERROR_DC_BUS_OVER_CURRENT = "0x00000010"
    ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION = "0x00000020"
    ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN = "0x00000040"
    ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE = "0x00000080"


class ODriveAxisError(StrEnum):
    AXIS_ERROR_NONE = "0x00000000"
    AXIS_ERROR_INVALID_STATE = "0x00000001"
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = "0x00000800"
    AXIS_ERROR_MIN_ENDSTOP_PRESSED = "0x00001000"
    AXIS_ERROR_MAX_ENDSTOP_PRESSED = "0x00002000"
    AXIS_ERROR_ESTOP_REQUESTED = "0x00004000"
    AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = "0x00020000"
    AXIS_ERROR_OVER_TEMP = "0x00040000"
    AXIS_ERROR_UNKNOWN_POSITION = "0x00080000"


class ODriveControllerError(StrEnum):
    CONTROLLER_ERROR_NONE = "0x00000000"
    CONTROLLER_ERROR_OVERSPEED = "0x00000001"
    CONTROLLER_ERROR_INVALID_INPUT_MODE = "0x00000002"
    CONTROLLER_ERROR_UNSTABLE_GAIN = "0x00000004"
    CONTROLLER_ERROR_INVALID_MIRROR_AXIS = "0x00000008"
    CONTROLLER_ERROR_INVALID_LOAD_ENCODER = "0x00000010"
    CONTROLLER_ERROR_INVALID_ESTIMATE = "0x00000020"
    CONTROLLER_ERROR_INVALID_CIRCULAR_RANGE = "0x00000040"
    CONTROLLER_ERROR_SPINOUT_DETECTED = "0x00000080"


class ODriveEncoderError(StrEnum):
    ENCODER_ERROR_NONE = "0x00000000"
    ENCODER_ERROR_UNSTABLE_GAIN = "0x00000001"
    ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH = "0x00000002"
    ENCODER_ERROR_NO_RESPONSE = "0x00000004"
    ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE = "0x00000008"
    ENCODER_ERROR_ILLEGAL_HALL_STATE = "0x00000010"
    ENCODER_ERROR_INDEX_NOT_FOUND_YET = "0x00000020"
    ENCODER_ERROR_ABS_SPI_TIMEOUT = "0x00000040"
    ENCODER_ERROR_ABS_SPI_COM_FAIL = "0x00000080"
    ENCODER_ERROR_ABS_SPI_NOT_READY = "0x00000100"
    ENCODER_ERROR_HALL_NOT_CALIBRATED_YET = "0x00000200"


class ODriveMotorError(StrEnum):
    MOTOR_ERROR_NONE = "0x00000000"
    MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = "0x00000001"
    MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = "0x00000002"
    MOTOR_ERROR_DRV_FAULT = "0x00000008"
    MOTOR_ERROR_CONTROL_DEADLINE_MISSED = "0x00000010"
    MOTOR_ERROR_MODULATION_MAGNITUDE = "0x00000080"
    MOTOR_ERROR_CURRENT_SENSE_SATURATION = "0x00000400"
    MOTOR_ERROR_CURRENT_LIMIT_VIOLATION = "0x00001000"
    MOTOR_ERROR_MODULATION_IS_NAN = "0x00010000"
    MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP = "0x00020000"
    MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP = "0x00040000"
    MOTOR_ERROR_TIMER_UPDATE_MISSED = "0x00080000"
    MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = "0x00100000"
    MOTOR_ERROR_CONTROLLER_FAILED = "0x00200000"
    MOTOR_ERROR_I_BUS_OUT_OF_RANGE = "0x00400000"
    MOTOR_ERROR_BRAKE_RESISTOR_DISARMED = "0x00800000"
    MOTOR_ERROR_SYSTEM_LEVEL = "0x01000000"
    MOTOR_ERROR_BAD_TIMING = "0x02000000"
    MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE = "0x04000000"
    MOTOR_ERROR_UNKNOWN_PHASE_VEL = "0x08000000"
    MOTOR_ERROR_UNKNOWN_TORQUE = "0x10000000"
    MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND = "0x20000000"
    MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT = "0x40000000"
    MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE = "0x80000000"
    MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND = "0x100000000"
    MOTOR_ERROR_UNKNOWN_GAINS = "0x200000000"
    MOTOR_ERROR_CONTROLLER_INITIALIZING = "0x400000000"
    MOTOR_ERROR_UNBALANCED_PHASES = "0x800000000"


class ODriveSensorlessEstimatorError(StrEnum):
    SENSORLESS_ESTIMATOR_ERROR_NONE = "0x00000000"
    SENSORLESS_ESTIMATOR_ERROR_UNSTABLE_GAIN = "0x00000001"
    SENSORLESS_ESTIMATOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT = "0x00000002"
