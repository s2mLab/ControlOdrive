""" This code helps to configure, calibrate and use an Odrive with a TSDZ2 motor."""

import time
import json
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

from enums import ControlMode, TrainingMode, control_modes_based_on_torque
from save_and_load import save


class OdriveEncoderHall:
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

        with open(hardware_and_security_path, "r") as hardware_and_security_file:
            self._hardware_and_security = json.load(hardware_and_security_file)

        self._watchdog_is_ready = False
        self._external_watchdog = external_watchdog
        self._watchdog_timeout = self._hardware_and_security["watchdog_timeout"]
        self._watchdog_feed_time = self._hardware_and_security["watchdog_feed_time"]
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        print("Odrive found")
        self._watchdog_is_ready = self.config_watchdog(enable_watchdog)

        self._reduction_ratio = self._hardware_and_security["reduction_ratio"]

        self._control_mode = ControlMode.STOP
        self._relative_pos = 0
        self._training_mode = TrainingMode.CONCENTRIC
        self.previous_control_mode = ControlMode.STOP

        self._gains_path = gains_path

        if file_path:
            self.file_path = file_path
        else:
            self.file_path = "XP/last_XP"
        self.first_save = True
        self.t0 = 0.0
        self.data = {
            "instruction": [],
            "time": [],
            "iq_setpoint": [],
            "iq_measured": [],
            "measured_torque": [],
            "user_torque": [],
            "velocity": [],
            "angle": [],
            "mechanical_power": [],
            "electrical_power": [],
            "user_power": [],
            "i_res": [],
            "vbus": [],
            "ibus": [],
        }

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

        self._hardware_and_security_configuration()
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
        # For position and velocity control
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
        # For position, velocity and torque control
        if current_gain is not None:
            self.odrv0.axis0.controller.config.current_gain = current_gain
        if current_integrator_gain is not None:
            self.odrv0.axis0.controller.config.current_integrator_gain = current_integrator_gain
        if bandwidth is not None:
            self.odrv0.axis0.encoder.config.bandwidth = bandwidth

        self.save_configuration()

    def _hardware_and_security_configuration(self):
        """
        Configures the settings linked to the hardware or the security not supposed to be changed by the user.
        """
        self.odrv0.config.enable_dc_bus_overvoltage_ramp = self._hardware_and_security["enable_dc_bus_overvoltage_ramp"]
        self.odrv0.config.dc_bus_overvoltage_ramp_start = self._hardware_and_security["dc_bus_overvoltage_ramp_start"]
        self.odrv0.config.dc_bus_overvoltage_ramp_end = self._hardware_and_security["dc_bus_overvoltage_ramp_end"]
        self.odrv0.config.gpio9_mode = self._hardware_and_security["gpio9_mode"]
        self.odrv0.config.gpio10_mode = self._hardware_and_security["gpio10_mode"]
        self.odrv0.config.gpio11_mode = self._hardware_and_security["gpio11_mode"]
        self.odrv0.config.max_regen_current = self._hardware_and_security["max_regen_current"]
        self.odrv0.config.dc_max_positive_current = self._hardware_and_security["dc_max_positive_current"]
        self.odrv0.config.dc_max_negative_current = self._hardware_and_security["dc_max_negative_current"]
        self.odrv0.config.enable_brake_resistor = self._hardware_and_security["enable_brake_resistor"]
        self.odrv0.config.brake_resistance = self._hardware_and_security["brake_resistance"]

        # Controller
        self.odrv0.axis0.controller.config.vel_limit = \
            self._hardware_and_security["pedals_vel_limit"] / self._reduction_ratio / 60  # tr/s
        self.odrv0.axis0.controller.config.vel_limit_tolerance = \
            self._hardware_and_security["vel_limit_tolerance"]  # tr/s

        # Encoder
        self.odrv0.axis0.encoder.config.mode = self._hardware_and_security["mode"]  # Mode of the encoder
        self.odrv0.axis0.encoder.config.cpr = self._hardware_and_security["cpr"]  # Count Per Revolution
        self.odrv0.axis0.encoder.config.calib_scan_distance = self._hardware_and_security["calib_scan_distance"]

        # Motor
        self.odrv0.axis0.motor.config.motor_type = self._hardware_and_security["motor_type"]
        self.odrv0.axis0.motor.config.pole_pairs = self._hardware_and_security["pole_pairs"]
        self.odrv0.axis0.motor.config.torque_constant = self._hardware_and_security["torque_constant"]
        self.odrv0.axis0.motor.config.calibration_current = self._hardware_and_security["calibration_current"]
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = \
            self._hardware_and_security["resistance_calib_max_voltage"]
        self.odrv0.axis0.motor.config.requested_current_range = self._hardware_and_security["requested_current_range"]
        self.odrv0.axis0.motor.config.current_control_bandwidth = \
            self._hardware_and_security["current_control_bandwidth"]
        self.odrv0.axis0.motor.config.current_lim = self._hardware_and_security["current_lim"]
        self.odrv0.axis0.motor.config.torque_lim = self._hardware_and_security["torque_lim"] * self._reduction_ratio

        # Velocity and acceleration limits
        self.odrv0.axis0.trap_traj.config.vel_limit = self.odrv0.axis0.controller.config.vel_limit
        self.odrv0.axis0.trap_traj.config.accel_limit = \
            self._hardware_and_security["pedals_accel_lim"] / self._reduction_ratio / 3600  # tr/s²
        self.odrv0.axis0.trap_traj.config.decel_limit = \
            self._hardware_and_security["pedals_accel_lim"] / self._reduction_ratio / 3600  # tr/s²

    def get_sign(self):
        """
        Set the sign of the rotation depending on the mode (eccentric or concentric).
        """
        if self._training_mode == TrainingMode.CONCENTRIC:
            return -1
        else:
            return 1

    def set_training_mode(self, mode: str):
        """
        Set the training mode to eccentric or concentric.

        Parameters
        ----------

        mode: TrainingMode
        """
        self._training_mode = TrainingMode(mode)

    def get_training_mode(self):
        """
        Get the training mode.

        Returns
        -------
        mode: TrainingMode
        """
        return self._training_mode

    def _check_ramp_rate(self, ramp_rate):
        """
        Check that the acceleration registered by the user is under the acceleration limit.

        Parameters
        ----------
        ramp_rate: float
            Acceleration of the pedals (tr/min²)
        """
        if abs(ramp_rate / self._reduction_ratio / 3600) > self.odrv0.axis0.trap_traj.config.accel_limit:
            raise ValueError(
                f"The acceleration limit is "
                f"{self.odrv0.axis0.trap_traj.config.accel_limit * self._reduction_ratio * 3600} "
                f"tr/min² for the pedals. "
                f"Acceleration specified: {abs(ramp_rate)} tr/min² for the pedals."
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

        print(self._relative_pos)
        print(self._relative_pos + turns / self._reduction_ratio)
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

    def velocity_control(self, velocity: float = 0.0, velocity_ramp_rate: float = 400.0):
        """
        Sets the motor to a given velocity in tr/min of the pedals with velocities ramped at each change.

        Parameters
        ----------
        velocity: float
            Targeted velocity in tr/min of the pedals.
        velocity_ramp_rate: float
            Velocity ramp rate in tr/min² of the pedals.
        """
        if abs(velocity / self._reduction_ratio / 60) > self.odrv0.axis0.controller.config.vel_limit:
            raise ValueError(
                f"The velocity limit is {self.odrv0.axis0.controller.config.vel_limit * self._reduction_ratio * 60} "
                f"tr/min for the pedals."
                f"Velocity specified: {abs(velocity)} tr/min for the pedals"
            )

        self._check_ramp_rate(velocity_ramp_rate)

        if self._control_mode != ControlMode.VELOCITY_CONTROL:
            self.stopping()
            self.stopped()
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        self.odrv0.axis0.controller.config.vel_ramp_rate = velocity_ramp_rate / 3600 / self._reduction_ratio
        self.odrv0.axis0.controller.input_vel = self.get_sign() * abs(velocity / 60 / self._reduction_ratio)

        if self._control_mode != ControlMode.VELOCITY_CONTROL:
            # Starts the motor if the previous control mode was not already `VELOCITY_CONTROL`
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self._control_mode = ControlMode.VELOCITY_CONTROL

    def vel_ecc_secu(self):
        self._control_mode = ControlMode.ECC_VEL_SECU
        self.odrv0.axis0.watchdog_feed()

        self.odrv0.axis0.controller.config.torque_ramp_rate = 2.0
        self.odrv0.axis0.controller.input_torque = 0.0

        self.odrv0.axis0.watchdog_feed()

        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

        self.odrv0.axis0.watchdog_feed()

        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_TORQUE_RAMP
        self.odrv0.axis0.controller.config.enable_torque_mode_vel_limit = True

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        t0 = time.time()

        while time.time() - t0 < 1.0:
            self.odrv0.axis0.watchdog_feed()

    def torque_control_init(self, input_torque_motor, torque_ramp_rate_motor, control_mode):
        """

        :return:
        """
        self.stopping()
        self.stopped()
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_TORQUE_RAMP
        self.odrv0.axis0.controller.config.enable_torque_mode_vel_limit = True

        self.odrv0.axis0.controller.config.torque_ramp_rate = torque_ramp_rate_motor
        self.odrv0.axis0.controller.input_torque = input_torque_motor

        # Starts the motor
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._control_mode = control_mode

    def torque_control(
            self,
            torque: float = 0.0,
            torque_ramp_rate: float = 2.0,
            resisting_torque_current: float = None):
        """
        Set the odrive in torque control, choose the torque and start the motor.

        Parameters
        ----------
        torque: float
            Torque (Nm) at the pedals.
        torque_ramp_rate: float
            Torque ramp rate (Nm/s) at the pedals.
        resisting_torque_current: float
            Current corresponding to the resisting torque at the pedals (A).
            If the variable `torque` is absolute, set resisting_torque to 0.0.
        """
        if abs(torque * self._reduction_ratio) > self.odrv0.axis0.motor.config.torque_lim:
            raise ValueError(
                f"The torque limit is {self.odrv0.axis0.motor.config.torque_lim / self._reduction_ratio} Nm."
                f"Torque specified: {torque} Nm"
            )

        if resisting_torque_current is None:
            resisting_torque_current = self._hardware_and_security["resisting_torque_current"]

        if torque != 0.0:
            torque = abs(torque) + self.odrv0.axis0.motor.config.torque_constant * resisting_torque_current \
                / self._reduction_ratio
        else:
            torque = 0.0

        torque_ramp_rate_motor = torque_ramp_rate * self._reduction_ratio
        input_torque_motor = - self.get_sign() * abs(torque * self._reduction_ratio)

        if self._control_mode != ControlMode.TORQUE_CONTROL:
            self.torque_control_init(input_torque_motor, torque_ramp_rate_motor, ControlMode.TORQUE_CONTROL)
        else:
            self.odrv0.axis0.controller.config.torque_ramp_rate = torque_ramp_rate_motor
            self.odrv0.axis0.controller.input_torque = input_torque_motor

    def stopping(
            self,
            vel_stop: float = 6.0,
            velocity_ramp_rate: float = 400.0,
            torque_stop: float = 2.0,
            torque_ramp_rate: float = 0.5
    ):
        """
        Starts the stopping sequence of the motor.

        Parameters
        ----------
        vel_stop: float
            The velocity at which the motor will be stopped if it was turning (tr/min of the pedals).
        velocity_ramp_rate: float
            The ramp_rate of the deceleration (tr/min² of the pedals).
        torque_stop: float
            The torque at which the motor will be stopped if it was turning (Nm of the pedals).
        torque_ramp_rate: float
            The ramp_rate of the deceleration (Nm/s of the pedals).
        """
        self._control_mode = ControlMode.STOPPING

        if vel_stop > self._hardware_and_security["maximal_velocity_stop"]:
            raise ValueError(
                f"The maximal velocity at which the motor can be stopped is "
                f"{self._hardware_and_security['maximal_velocity_stop']} tr/min for the pedals."
                f"Stop velocity specified: {abs(vel_stop)} tr/min for the pedals"
            )

        self._check_ramp_rate(velocity_ramp_rate)

        if (self._control_mode == ControlMode.VELOCITY_CONTROL or self._control_mode == ControlMode.POSITION_CONTROL) \
                and abs(self.odrv0.axis0.encoder.vel_estimate) > vel_stop / 60 / self._reduction_ratio:
            # Gently slows down the motor.
            self.odrv0.axis0.controller.config.vel_ramp_rate = velocity_ramp_rate / 3600 / self._reduction_ratio
            self.odrv0.axis0.controller.input_vel = 0.0

        if (self._control_mode in control_modes_based_on_torque) \
                and abs(self.get_measured_torque()) > torque_stop:
            # Gently slows down the motor.
            self.odrv0.axis0.controller.config.torque_ramp_rate = torque_ramp_rate * self._reduction_ratio
            self.odrv0.axis0.controller.input_torque = 0.0

    def stopped(self):
        """
        Running until the motor is fully stopped. Can be executed in another thread or process.
        """
        if self.previous_control_mode == ControlMode.VELOCITY_CONTROL:
            while abs(self.get_velocity()) > 6.0:
                pass
        elif self.previous_control_mode in control_modes_based_on_torque:
            while abs(self.get_measured_torque()) > 0.5:
                pass

        # Stops the motor
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis0.controller.input_vel = 0.0
        self.odrv0.axis0.controller.input_torque = 0.0
        self._control_mode = ControlMode.STOP

        return True

    def stop(
        self,
        vel_stop: float = 6.0,
        velocity_ramp_rate: float = 400.0,
        torque_stop: float = 2.0,
        torque_ramp_rate: float = 0.5
    ):
        """
        Stops the motor gently.

        Parameters
        ----------
        vel_stop: float
            The velocity at which the motor will be stopped if it was turning (tr/min of the pedals).
        velocity_ramp_rate: float
            The ramp_rate of the deceleration (tr/min² of the pedals).
        torque_stop: float
            The torque at which the motor will be stopped if it was turning (Nm of the pedals).
        torque_ramp_rate: float
            The ramp_rate of the deceleration (Nm/s of the pedals).
        """
        self.stopping(vel_stop, velocity_ramp_rate, torque_stop, torque_ramp_rate)
        self.stopped()

    def get_reduction_ratio(self):
        return self._reduction_ratio

    def get_control_mode(self):
        return self._control_mode

    def get_angle(self):
        """
        Returns the estimated angle in degrees. A calibration is needed to know the 0.
        """
        return ((self.odrv0.axis0.encoder.pos_estimate - self._relative_pos) * self._reduction_ratio * 360) % 360

    def get_velocity(self) -> float:
        """
        Returns the estimated velocity of the pedals in tr/min.
        """
        return self.odrv0.axis0.encoder.vel_estimate * self._reduction_ratio * 60

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
        return self.get_user_torque() * self.get_velocity() * 2 * np.pi / 60

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

    def get_i_res(self, a: float = 0.83504094, b: float = 0.00483589, c: float = 3.25038644):
        """
        Returns the motor current in A corresponding to the effort produced by the user.
        a, b and c are coefficient that have been obtained by a curve fitting done scipy.
        The angle should have been taken into account but since there is a lot of noise on the estimated velocity, the
        influence of the angle was deemed negligible.
        """
        vel = self.odrv0.axis0.encoder.vel_estimate * self._reduction_ratio
        if vel == 0.0:
            return 0.0
        else:
            return - a * vel / abs(vel) * abs(vel) ** (1 / c) + b

    def get_measured_torque(self):
        """
        Returns the measured torque.
        """
        return self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.motor.current_control.Iq_measured \
            / self._reduction_ratio

    def get_user_torque(self):
        """
        Returns the measured user torque (the resisting torque has been subtracted).
        """
        i_measured = self.odrv0.axis0.motor.current_control.Iq_measured
        velocity = self.odrv0.axis0.encoder.vel_estimate
        if velocity != 0.0:
            dyn_resisting_current = - 0.8 * velocity / abs(velocity) * abs(velocity)**(1/3.2)
            i_user = i_measured - dyn_resisting_current
        else:
            i_user = i_measured
        if abs(i_user) <= self._hardware_and_security["resisting_torque_current"]:
            return 0.0
        else:
            sign = - i_user / abs(i_user)
            return self.odrv0.axis0.motor.config.torque_constant * \
                (i_user + sign * self._hardware_and_security["resisting_torque_current"]) / self._reduction_ratio

    def save_data_to_file(self, file_path: str, spin_box: float = None, instruction: float = None):
        """
        Saves data.
        """
        if self.first_save:
            self.t0 = time.time()
            self.first_save = False

        data = {
            "spin_box": spin_box,
            "instruction": instruction,
            "time": time.time() - self.t0,
            "iq_setpoint": self.get_iq_setpoint(),
            "iq_measured": self.get_iq_measured(),
            "measured_torque": self.get_measured_torque(),
            "user_torque": self.get_user_torque(),
            "velocity": self.get_velocity(),
            "angle": self.get_angle(),
            "mechanical_power": self.get_mechanical_power(),
            "electrical_power": self.get_electrical_power(),
            "user_power": self.get_user_power(),
            "i_res": self.get_i_res(),
            "vbus": self.odrv0.vbus_voltage,
            "ibus": self.odrv0.ibus,
            "error": self.odrv0.error,
            "axis_error": self.odrv0.axis0.error,
            "controller_error": self.odrv0.axis0.controller.error,
            "encoder_error": self.odrv0.axis0.encoder.error,
            "motor_error": self.odrv0.axis0.motor.error,
            "sensorless_estimator_error": self.odrv0.axis0.sensorless_estimator.error,
            "state": self.odrv0.axis0.current_state,
            "control_mode": self._control_mode.value,
            "training_mode": self._training_mode.value,
            "resistor_current": self.odrv0.brake_resistor_current,
            "brake_resistor_saturated": self.odrv0.brake_resistor_saturated,
            "brake_resistor_armed": self.odrv0.brake_resistor_armed,
        }

        save(data, file_path)