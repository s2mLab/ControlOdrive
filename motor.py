""" This code helps to configure, calibrate and use an Odrive with a TSDZ2 motor."""

import time
import json
import copy
import threading
import numpy as np

import odrive
import fibre.libfibre
from odrive.enums import (
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_TRAP_TRAJ,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_TORQUE_RAMP,
    AXIS_ERROR_NONE,
    CONTROLLER_ERROR_NONE,
    ENCODER_ERROR_NONE,
    MOTOR_ERROR_NONE,
    SENSORLESS_ESTIMATOR_ERROR_NONE,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED,
)

from enums import ControlMode, DirectionMode, control_modes_based_on_torque, control_modes_based_on_cadence
from save_and_load import save

from motor_computations import MotorComputations


class MotorController(MotorComputations):
    """
    Represents a motor controlled by an Odrive with the integrated Hall encoder. This script has been written for one
    Odrive and one motor TSDZ2 wired on the axis0 of the Odrive. If a motor happens to be wired on axis1 the all the
    occurrences of `axis0` would need to be set to `axis1`. If the user wants to connect a second Odrive, the script
    would need to be adapted with `odrv1`.
    """

    def __init__(
            self,
            enable_watchdog=True,
            external_watchdog: bool = False,
            hardware_and_security_path: str = "parameters/hardware_and_security.json",
            gains_path: str = "parameters/gains.json",
            file_path: str = None,
    ):
        super(MotorController, self).__init__()
        self._watchdog_is_ready = False
        self._external_watchdog = external_watchdog
        self._watchdog_timeout = self.hardware_and_security["watchdog_timeout"]
        self._watchdog_feed_time = self.hardware_and_security["watchdog_feed_time"]
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        print("Odrive found")
        self._watchdog_is_ready = self.config_watchdog(enable_watchdog)

        self._control_mode = ControlMode.STOP
        self._relative_pos = 0
        self._direction = DirectionMode.FORWARD
        self.previous_control_mode = ControlMode.STOP

        self._gains_path = gains_path

        if file_path:
            self.file_path = file_path
        else:
            self.file_path = "XP/last_XP"
        self.first_save = True
        self.t0 = 0.0

    def erase_configuration(self):
        """
        Resets all config variables to their default values and reboots the controller.
        fibre.libfibre.ObjectLostError "[LEGACY_OBJ] protocol failed with 3 - propagating error to application" is not
        an issue.
        """
        try:
            self.odrv0.erase_configuration()
        except fibre.libfibre.ObjectLostError:
            pass
        self.odrv0 = odrive.find_any()
        try:
            self.odrv0.reboot()
        except fibre.libfibre.ObjectLostError:
            pass
        self.odrv0 = odrive.find_any()

    def save_configuration(self):
        """
        Saves the current configuration to non-volatile memory and reboots the board.
        fibre.libfibre.ObjectLostError "[LEGACY_OBJ] protocol failed with 3 - propagating error to application" is not
        an issue.
        """

        try:
            self.odrv0.save_configuration()
        except fibre.libfibre.ObjectLostError:
            pass
        self.odrv0 = odrive.find_any()
        try:
            self.odrv0.reboot()
        except fibre.libfibre.ObjectLostError:
            pass
        self.odrv0 = odrive.find_any()

    def config_watchdog(self, enable_watchdog: bool, watchdog_timeout: float = None):
        """
        Configures a watchdog. If the odrive does not receive a watchdog before the time given in watchdog_timeout, it
        will disconnect.

        Parameters
        ----------
        enable_watchdog: bool
            Indicates if the user wants to enable the watchdog (True) or not (False)
        watchdog_timeout: float
            Maximum duration since the last feeding in s. If the watchdog has not been fed for more than
            `watchdog_timeout` the Odrive unable the motor.

        Returns
        -------
        watchdog_is_ready: bool
            Indicates if the watchdog is enabled or not
        """
        self.odrv0.axis0.config.enable_watchdog = False
        if self.odrv0.axis0.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED:
            self.odrv0.axis0.error = AXIS_ERROR_NONE
        if enable_watchdog:
            if watchdog_timeout:
                self._watchdog_timeout = watchdog_timeout
            if not self._external_watchdog:
                watchdog = threading.Thread(target=self._watchdog_feed, name="Watchdog", daemon=True)
                watchdog.start()
            print("Waiting to enable the watchdog...")
            self.odrv0.axis0.config.watchdog_timeout = self._watchdog_timeout
            self.odrv0.axis0.config.enable_watchdog = True
            # Some errors may occur at the instant the watchdog is enabled, the sleep time is to let these errors pass.
            # As all the code is on the same thread (except for `_watchdog_feed`), not anything else will happen until
            # the watchdog is fully enabled.
            time.sleep(3 * self._watchdog_timeout)  # 3 was chosen arbitrarily
            # Clears only the `AXIS_ERROR_WATCHDOG_TIMER_EXPIRED`, if there is another error on the axis, we are not
            # able to conserve know it as `odrv0.axis0.error` can only be on one state.
            if self.odrv0.axis0.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED:
                self.odrv0.axis0.error = AXIS_ERROR_NONE
            self.odrv0.clear_errors()
            print("Watchdog enabled")
            return True
        else:
            return False

    def _watchdog_feed(self):
        """
        Feeds the watchdog. To be called by a daemon thread.
        """
        while True:
            self.odrv0.axis0.watchdog_feed()
            # self.save_data_to_file(self.file_path)
            time.sleep(self._watchdog_feed_time)

    def calibration(self, mechanical_load: bool = True):
        """
        Calibrates the Odrive. It is advised to do one first full calibration under no mechanical load and to do a
        controller calibration under mechanical load each time the Odrive is turned on.

        Parameters
        ----------
        mechanical_load: bool
            Indicates if the motor is under mechanical load (True) or not (False).
        """
        print("Start motor calibration")

        if mechanical_load:
            # Starts only an encoder calibration because the motor is under mechanical load
            self.odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        else:
            # Starts a full calibration because the motor is not under mechanical load
            self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        # Waits for the calibration to finish.
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            pass

        if self.has_error():
            raise RuntimeError("Error with configuration and/or calibration. Check odrivetool -> dump_errors(odrv0)")

        # Confirms the configuration and calibration. Allows to save the configuration after reboot.
        self.odrv0.axis0.encoder.config.pre_calibrated = True
        self.odrv0.axis0.motor.config.pre_calibrated = True
        self.save_configuration()

        print("Calibration done")

    def has_error(self):
        """
        Indicates if one or several errors has been detected (True) or not (False).
        """
        return not (
                self.odrv0.error == 0
                and self.odrv0.axis0.error == AXIS_ERROR_NONE
                and self.odrv0.axis0.controller.error == CONTROLLER_ERROR_NONE
                and self.odrv0.axis0.encoder.error == ENCODER_ERROR_NONE
                and self.odrv0.axis0.motor.error == MOTOR_ERROR_NONE
                and self.odrv0.axis0.sensorless_estimator.error == SENSORLESS_ESTIMATOR_ERROR_NONE
        )

    def configuration(self):
        """
        Configures the Odrive.
        """
        print("Configuration for HALL encoder")

        self.hardware_and_security_configuration()
        self.gains_configuration(custom=False)

        print("Configuration done")

    def gains_configuration(
            self,
            custom: bool,
            pos_gain: float = None,
            k_vel_gain: float = None,
            k_vel_integrator_gain: float = None,
            current_gain: float = None,
            current_integrator_gain: float = None,
            bandwidth: float = None,
    ):
        """
        custom: bool
            Indicates if the user wants to use the previously calculated gains saved in the .json (False) or to modify
            manually the gains (True).
        pos_gain: float
        k_vel_gain: float
        k_vel_integrator_gain: float
        current_gain: float
        current_integrator_gain: float
        bandwidth: float
        """
        if not custom:
            with open(self._gains_path, "r") as gain_file:
                gains = json.load(gain_file)

            pos_gain = gains["pos_gain"]
            k_vel_gain = gains["k_vel_gain"]
            k_vel_integrator_gain = gains["k_vel_integrator_gain"]
            current_gain = None
            current_integrator_gain = None
            bandwidth = gains["bandwidth"]

        # Gains
        # For position control only
        if pos_gain is not None:
            self.odrv0.axis0.controller.config.pos_gain = pos_gain
        # For position and cadence control
        if k_vel_gain is not None:
            self.odrv0.axis0.controller.config.vel_gain = (
                    k_vel_gain * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.encoder.config.cpr
            )
        if k_vel_integrator_gain is not None:
            self.odrv0.axis0.controller.config.vel_integrator_gain = (
                    k_vel_integrator_gain
                    * self.odrv0.axis0.motor.config.torque_constant
                    * self.odrv0.axis0.encoder.config.cpr
            )
        # For position, cadence and torque control
        if current_gain is not None:
            self.odrv0.axis0.controller.config.current_gain = current_gain
        if current_integrator_gain is not None:
            self.odrv0.axis0.controller.config.current_integrator_gain = current_integrator_gain
        if bandwidth is not None:
            self.odrv0.axis0.encoder.config.bandwidth = bandwidth

        self.save_configuration()

    def hardware_and_security_configuration(self):
        """
        Configures the settings linked to the hardware or the security not supposed to be changed by the user.
        """
        self.odrv0.config.enable_dc_bus_overvoltage_ramp = self.hardware_and_security["enable_dc_bus_overvoltage_ramp"]
        self.odrv0.config.dc_bus_overvoltage_ramp_start = self.hardware_and_security["dc_bus_overvoltage_ramp_start"]
        self.odrv0.config.dc_bus_overvoltage_ramp_end = self.hardware_and_security["dc_bus_overvoltage_ramp_end"]
        self.odrv0.config.gpio9_mode = self.hardware_and_security["gpio9_mode"]
        self.odrv0.config.gpio10_mode = self.hardware_and_security["gpio10_mode"]
        self.odrv0.config.gpio11_mode = self.hardware_and_security["gpio11_mode"]
        self.odrv0.config.max_regen_current = self.hardware_and_security["max_regen_current"]
        self.odrv0.config.dc_max_positive_current = self.hardware_and_security["dc_max_positive_current"]
        self.odrv0.config.dc_max_negative_current = self.hardware_and_security["dc_max_negative_current"]
        self.odrv0.config.enable_brake_resistor = self.hardware_and_security["enable_brake_resistor"]
        self.odrv0.config.brake_resistance = self.hardware_and_security["brake_resistance"]

        # Controller
        self.odrv0.axis0.controller.config.vel_limit = \
            self.hardware_and_security["pedals_vel_limit"] / self._reduction_ratio / 60  # tr/s
        self.odrv0.axis0.controller.config.vel_limit_tolerance = \
            self.hardware_and_security["vel_limit_tolerance"]  # tr/s

        # Encoder
        self.odrv0.axis0.encoder.config.mode = self.hardware_and_security["mode"]  # Mode of the encoder
        self.odrv0.axis0.encoder.config.cpr = self.hardware_and_security["cpr"]  # Count Per Revolution
        self.odrv0.axis0.encoder.config.calib_scan_distance = self.hardware_and_security["calib_scan_distance"]

        # Motor
        self.odrv0.axis0.motor.config.motor_type = self.hardware_and_security["motor_type"]
        self.odrv0.axis0.motor.config.pole_pairs = self.hardware_and_security["pole_pairs"]
        self.odrv0.axis0.motor.config.torque_constant = self.hardware_and_security["torque_constant"]
        self.odrv0.axis0.motor.config.calibration_current = self.hardware_and_security["calibration_current"]
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = \
            self.hardware_and_security["resistance_calib_max_voltage"]
        self.odrv0.axis0.motor.config.requested_current_range = self.hardware_and_security["requested_current_range"]
        self.odrv0.axis0.motor.config.current_control_bandwidth = \
            self.hardware_and_security["current_control_bandwidth"]
        self.odrv0.axis0.motor.config.current_lim = self.hardware_and_security["current_lim"]
        self.odrv0.axis0.motor.config.torque_lim = self.hardware_and_security["torque_lim"] * self._reduction_ratio

        # cadence and acceleration limits
        self.odrv0.axis0.trap_traj.config.vel_limit = self.odrv0.axis0.controller.config.vel_limit
        self.odrv0.axis0.trap_traj.config.accel_limit = \
            self.hardware_and_security["pedals_accel_lim"] / self._reduction_ratio / 60  # tr/s²
        self.odrv0.axis0.trap_traj.config.decel_limit = \
            self.hardware_and_security["pedals_accel_lim"] / self._reduction_ratio / 60  # tr/s²

    def get_sign(self):
        """
        Set the sign of the rotation depending on the mode (eccentric or concentric).
        # TODO: track ecc cons
        """
        if self._direction == DirectionMode.FORWARD:
            return -1
        else:
            return 1

    def set_direction(self, mode: str):
        """
        Set the direction to forward or reversed.

        Parameters
        ----------

        mode: DirectionMode
        """
        self._direction = DirectionMode(mode)

    def get_direction(self):
        """
        Get the direction.

        Returns
        -------
        mode: DirectionMode
        """
        return self._direction

    def _check_ramp_rate(self, ramp_rate):
        """
        Check that the acceleration registered by the user is under the acceleration limit.

        Parameters
        ----------
        ramp_rate: float
            Acceleration of the pedals (rpm/s)
        """
        if abs(ramp_rate / self._reduction_ratio / 60) > self.odrv0.axis0.trap_traj.config.accel_limit:
            raise ValueError(
                f"The acceleration limit is "
                f"{self.odrv0.axis0.trap_traj.config.accel_limit * self._reduction_ratio * 3600} "
                f"rpm² for the pedals. "
                f"Acceleration specified: {abs(ramp_rate)} rpm/s for the pedals."
            )

    def turns_control(self, turns: float = 0.0):
        """
        Makes the motors turn for the indicated number of turns.

        Parameters
        ----------
        turns: float
            The number of turns the motor will do.
        """
        if self._control_mode != ControlMode.POSITION_CONTROL:
            self.stop()
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
            self._relative_pos = self.odrv0.axis0.encoder.pos_estimate
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self._control_mode = ControlMode.POSITION_CONTROL

        self.odrv0.axis0.controller.input_pos = self._relative_pos + turns / self._reduction_ratio

    def zero_position_calibration(self):
        """
        Calibration for the 0 deg.
        """
        self._relative_pos = self.odrv0.axis0.encoder.pos_estimate

    def position_control(self, angle: float = 0.0):
        """
        Leads the motors to the indicated angle.

        Parameters
        ----------
        angle: float
            The angle the motor must go to ([-180.0, 360.0] deg)
        """
        if angle > 360.0 or angle < -180.0:
            raise ValueError(
                f"The angle specified must be between -180.0 deg and 360.0 deg. Angle specified: {angle} deg"
            )

        if self._control_mode != ControlMode.POSITION_CONTROL:
            self.stop()
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
            # Starts the motor
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self._control_mode = ControlMode.POSITION_CONTROL

        self.odrv0.axis0.controller.input_pos = self._relative_pos + angle / 360 / self._reduction_ratio

    def cadence_control(
            self,
            cadence: float = 0.0,
            cadence_ramp_rate: float = 5,
            control_mode: ControlMode = ControlMode.CADENCE_CONTROL,
    ):
        """
        Sets the motor to a given cadence in rpm of the pedals with velocities ramped at each change.

        Parameters
        ----------
        cadence: float
            Targeted cadence in rpm of the pedals.
        cadence_ramp_rate: float
            cadence ramp rate in rpm/s of the pedals.
        control_mode: ControlMode
            Control mode of the motor.
        """
        cadence = abs(cadence)
        if cadence / self._reduction_ratio / 60 > self.odrv0.axis0.controller.config.vel_limit:
            raise ValueError(
                f"The cadence limit is {self.odrv0.axis0.controller.config.vel_limit * self._reduction_ratio * 60} "
                f"rpm for the pedals."
                f"cadence specified: {cadence} rpm for the pedals"
            )

        self._check_ramp_rate(cadence_ramp_rate)

        if self._control_mode not in control_modes_based_on_cadence:
            self.stopping()
            self.stopped()
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        self.odrv0.axis0.controller.config.vel_ramp_rate = cadence_ramp_rate / 60 / self._reduction_ratio
        self.odrv0.axis0.controller.input_vel = self.get_sign() * cadence / 60 / self._reduction_ratio

        if self._control_mode not in control_modes_based_on_cadence:
            # Starts the motor if the previous control mode was not already `cadence_CONTROL`
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self._control_mode = control_mode

        return - self.get_sign() * cadence

    def torque_control(
            self,
            torque: float = 0.0,
            torque_ramp_rate: float = 2.0,
            resisting_torque: float = None,
            control_mode: ControlMode = ControlMode.TORQUE_CONTROL,
    ):
        """
        Set the odrive in torque control, choose the torque and start the motor.

        Parameters
        ----------
        torque: float
            Torque (Nm) at the pedals.
        torque_ramp_rate: float
            Torque ramp rate (Nm/s) at the pedals.
        resisting_torque: float
            Resisting torque at the pedals (Nm).
            If the variable `torque` is absolute, set resisting_torque to 0.0.
        control_mode: ControlMode
            Control mode to use.

        Returns
        -------
        The input torque (Nm) at the pedals.
        """
        # If the user is not pedaling yet or if he has stopped pedaling, the motor is stopped.
        if ((self._direction == DirectionMode.FORWARD and self.odrv0.axis0.encoder.vel_estimate >= 0)
                or (self._direction == DirectionMode.REVERSE and self.odrv0.axis0.encoder.vel_estimate <= 0)):
            input_torque_motor = 0.0
            torque_ramp_rate_motor = 100.0
        # If the user is pedaling, the torque and torque_ramp values have to be translated to the motor.
        else:
            # TODO: Check if the torque ramp rate is correct
            torque_ramp_rate_motor = torque_ramp_rate * self._reduction_ratio

            if abs(torque * self._reduction_ratio) > self.odrv0.axis0.motor.config.torque_lim:
                raise ValueError(
                    f"The torque limit is {self.odrv0.axis0.motor.config.torque_lim / self._reduction_ratio} Nm."
                    f"Torque specified: {torque} Nm"
                )

            if resisting_torque is None:
                resisting_torque = self.get_resisting_torque()

            if torque != 0.0:
                torque = abs(torque) - self.get_sign() * resisting_torque
            else:
                torque = 0.0

            input_torque_motor = - self.get_sign() * abs(torque * self._reduction_ratio)

        # The motor can be controlled with the computed values
        if self._control_mode not in control_modes_based_on_torque:
            self.stopping()
            self.stopped()
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_TORQUE_RAMP
            self.odrv0.axis0.controller.config.enable_torque_mode_vel_limit = True

            self.odrv0.axis0.controller.config.torque_ramp_rate = torque_ramp_rate_motor
            self.odrv0.axis0.controller.input_torque = input_torque_motor

            # Starts the motor
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        else:
            self.odrv0.axis0.controller.config.torque_ramp_rate = torque_ramp_rate_motor
            self.odrv0.axis0.controller.input_torque = input_torque_motor

        # In case the previous control mode was based on torque control but was
        # not `TORQUE_CONTROL`, self._control_mode is updated.
        self._control_mode = control_mode

        return - input_torque_motor / self._reduction_ratio  # Nm at the pedals

    def concentric_power_control(
            self,
            power: float = 0.0,
            torque_ramp_rate: float = 2.0,
            resisting_torque: float = None):
        """
        # TODO add docstring in all to explain to call in a thread.
        Parameters
        ----------
        power: float
            Power (W) at the pedals.
        torque_ramp_rate: float
            Torque ramp rate (Nm/s) at the pedals.
        resisting_torque: float
            Resisting torque at the pedals (Nm).
            If the variable `torque` is absolute, set resisting_torque to 0.0.

        Returns
        -------
        The input torque (Nm) at the pedals.
        """
        cadence = abs(self.odrv0.axis0.encoder.vel_estimate * self._reduction_ratio * 2 * np.pi)  # rad/s
        if cadence == 0:
            return self.torque_control(0.0, torque_ramp_rate, resisting_torque, ControlMode.CONCENTRIC_POWER_CONTROL)
        else:
            return self.torque_control(
                min(abs(power) / cadence, self.hardware_and_security["torque_lim"]),
                torque_ramp_rate,
                resisting_torque,
                ControlMode.CONCENTRIC_POWER_CONTROL,
            )

    def eccentric_power_control(
            self,
            power: float = 0.0,
            cadence_ramp_rate: float = 5.0,
            cadence_max: float = 50.0):
        """
        Parameters
        ----------
        power: float
            Power (W) at the pedals.
        cadence_ramp_rate: float
            cadence ramp rate (rpm/s) at the pedals.
        cadence_max: float
            Maximum cadence rpm at the pedals, if no torque is applied or if the torque < power / cadence_max.

        Returns
        -------
        The input torque (Nm) at the pedals.
        """
        torque = self.get_user_torque()

        # If the user is not forcing against the motor, the motor goes to the maximum cadence.
        if (self._direction == DirectionMode.REVERSE and torque >= 0)\
                or (self._direction == DirectionMode.FORWARD and torque <= 0):
            self.cadence_control(cadence_max, cadence_ramp_rate, ControlMode.ECCENTRIC_POWER_CONTROL)
            return np.inf  # So we know that the user is not forcing.
        else:
            cadence = min(abs(power / torque) / 2 / np.pi * 60, cadence_max)
            return self.cadence_control(cadence, cadence_ramp_rate, ControlMode.ECCENTRIC_POWER_CONTROL)

    def linear_control(
            self,
            linear_coeff: float = 0.0,
            torque_ramp_rate: float = 2.0,
            resisting_torque: float = None):
        """
        Parameters
        ----------
        linear_coeff: float
            Linear coefficient (Nm/rpm) at the pedals.
        torque_ramp_rate: float
            Torque ramp rate (Nm/s) at the pedals.
        resisting_torque: float
            Resisting torque at the pedals (Nm).
            If the variable `torque` is absolute, set resisting_torque to 0.0.

        Returns
        -------
        The input torque (Nm) at the pedals.
        """
        cadence = abs(self.get_cadence())  # rpm
        return self.torque_control(
            min(cadence * abs(linear_coeff), self.hardware_and_security["torque_lim"]),
            torque_ramp_rate,
            resisting_torque,
            ControlMode.LINEAR_CONTROL,
        )

    def stopping(
            self,
            cadence_ramp_rate: float = 30,
    ):
        """
        Starts the stopping sequence of the motor.

        Parameters
        ----------
        cadence_ramp_rate: float
            The ramp_rate of the deceleration (rpm/s of the pedals).
        """
        self.previous_control_mode = copy.deepcopy(self._control_mode)

        self._check_ramp_rate(cadence_ramp_rate)

        if (self.previous_control_mode in control_modes_based_on_cadence
                or self.previous_control_mode == ControlMode.POSITION_CONTROL):
            # Gently slows down the motor.
            self.odrv0.axis0.controller.config.vel_ramp_rate = cadence_ramp_rate / 60 / self._reduction_ratio
            self.odrv0.axis0.controller.input_vel = 0.0

        self._control_mode = ControlMode.STOPPING

    def stopped(self):
        """
        Running until the motor is fully stopped. Can be executed in another thread or process.

        Returns
        -------
        True when the motor has stopped.
        """

        # Stops the motor
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis0.controller.input_vel = 0.0
        self.odrv0.axis0.controller.input_torque = 0.0
        self._control_mode = ControlMode.STOP

        return True

    def stop(
        self,
        vel_stop: float = 10.0,
        cadence_ramp_rate: float = 30,
    ):
        """
        Stops the motor gently.

        Parameters
        ----------
        vel_stop: float
            The cadence at which the motor will be stopped if it was turning (rpm of the pedals).
        cadence_ramp_rate: float
            The ramp_rate of the deceleration (rpm/s of the pedals).
        """
        if vel_stop > self.hardware_and_security["maximal_cadence_stop"]:
            raise ValueError(
                f"The maximal cadence at which the motor can be stopped is "
                f"{self.hardware_and_security['maximal_cadence_stop']} rpm for the pedals."
                f"Stop cadence specified: {abs(vel_stop)} rpm for the pedals"
            )

        self.stopping(cadence_ramp_rate)

        while abs(self.get_cadence()) > vel_stop:
            pass

        self.stopped()

    def get_control_mode(self):
        """
        Returns the current control mode.
        """
        return self._control_mode

    def get_angle(self):
        """
        Returns the estimated angle in degrees. A calibration is needed to know the 0.
        """
        return self.compute_angle(self.get_turns())

    def get_turns(self):
        """
        Returns the estimated number of turns.
        """
        return - (self.odrv0.axis0.encoder.pos_estimate - self._relative_pos) * self._reduction_ratio

    def get_cadence(self) -> float:
        """
        Returns the estimated cadence of the pedals in rpm.
        """
        return self.compute_cadence(self.odrv0.axis0.encoder.vel_estimate)

    def get_electrical_power(self):
        """
        Returns the electrical power in W.
        """
        return self.odrv0.axis0.controller.electrical_power

    def get_mechanical_power(self):
        """
        Returns the mechanical power in W.
        """
        return self.odrv0.axis0.controller.mechanical_power

    def get_user_power(self):
        """
        Returns the user mechanical power in W.
        """
        return self.compute_user_torque(self.get_user_torque(), self.get_cadence())

    def get_iq_setpoint(self):
        """
        Returns the commanded motor current in A.
        """
        return self.odrv0.axis0.motor.current_control.Iq_setpoint

    def get_iq_measured(self):
        """
        Returns the measured motor current in A.
        """
        return self.odrv0.axis0.motor.current_control.Iq_measured

    def get_measured_torque(self):
        """
        Returns the measured torque.
        """
        return - self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.motor.current_control.Iq_measured \
            / self._reduction_ratio

    def get_motor_torque(self):
        """
        Returns the measured torque.
        """
        return self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.motor.current_control.Iq_measured \
            / self._reduction_ratio

    def get_resisting_torque(self):
        """
        Returns the resisting torque.
        """
        return self.compute_resisting_torque(
            self.odrv0.axis0.motor.current_control.Iq_measured,
            self.odrv0.axis0.encoder.vel_estimate,)

    def get_user_torque(self):
        """
        Returns the measured user torque (the resisting torque is subtracted from the motor_torque).
        """
        return self.compute_user_torque(
            self.odrv0.axis0.motor.current_control.Iq_measured,
            self.odrv0.axis0.encoder.vel_estimate
        )

    def save_data_to_file(
            self,
            file_path: str,
            spin_box: float = None,
            instruction: float = None,
            ramp_instruction: float = None,
            comment: str = "",
            stopwatch: float = 0.0,
            lap: float = 0.0,
    ):
        """
        Saves data.
        """
        if self.first_save:
            self.t0 = time.time()
            self.first_save = False

        data = {
            "comments": comment,
            "stopwatch": stopwatch,
            "lap": lap,
            "spin_box": spin_box,
            "instruction": instruction,
            "ramp_instruction": ramp_instruction,
            "time": time.time() - self.t0,
            "user_torque": self.get_user_torque(),
            "cadence": self.get_cadence(),
            "angle": self.get_angle(),
            "turns": self.get_turns(),
            "user_power": self.get_user_power(),
            "error": self.odrv0.error,
            "axis_error": self.odrv0.axis0.error,
            "controller_error": self.odrv0.axis0.controller.error,
            "encoder_error": self.odrv0.axis0.encoder.error,
            "motor_error": self.odrv0.axis0.motor.error,
            "sensorless_estimator_error": self.odrv0.axis0.sensorless_estimator.error,
            "can_error": self.odrv0.can.error,
            "state": self.odrv0.axis0.current_state,
            "control_mode": self._control_mode.value,
            "direction": self._direction.value,
            "iq_setpoint": self.get_iq_setpoint(),
            "iq_measured": self.get_iq_measured(),
            "measured_torque": self.get_measured_torque(),
            "motor_torque": self.get_motor_torque(),
            "resisting_torque": self.get_resisting_torque(),
            "mechanical_power": self.get_mechanical_power(),
            "electrical_power": self.get_electrical_power(),
            "vbus": self.odrv0.vbus_voltage,
            "ibus": self.odrv0.ibus,
            "resistor_current": self.odrv0.brake_resistor_current,
            "brake_resistor_saturated": self.odrv0.brake_resistor_saturated,
            "brake_resistor_armed": self.odrv0.brake_resistor_armed,

        }

        save(data, file_path)

    def minimal_save_data_to_file(
            self,
            file_path: str,
            spin_box: float = None,
            instruction: float = None,
            ramp_instruction: float = None,
            comment: str = "",
            stopwatch: float = 0.0,
            lap: float = 0.0,
    ):
        """
        Saves data.
        """
        if self.first_save:
            self.t0 = time.time()
            self.first_save = False

        data = {
            "time": time.time() - self.t0,

            "spin_box": spin_box,
            "instruction": instruction,
            "ramp_instruction": ramp_instruction,

            "comments": comment,
            "stopwatch": stopwatch,
            "lap": lap,

            "state": self.odrv0.axis0.current_state,
            "control_mode": self._control_mode.value,
            "direction": self._direction.value,

            "vel_estimate": self.odrv0.axis0.encoder.vel_estimate,
            "turns": self.get_turns(),
            "iq_measured": self.odrv0.axis0.motor.current_control.Iq_measured,

            "error": self.odrv0.error,
            "axis_error": self.odrv0.axis0.error,
            "controller_error": self.odrv0.axis0.controller.error,
            "encoder_error": self.odrv0.axis0.encoder.error,
            "motor_error": self.odrv0.axis0.motor.error,
            "sensorless_estimator_error": self.odrv0.axis0.sensorless_estimator.error,
            "can_error": self.odrv0.can.error,
        }

        save(data, file_path)