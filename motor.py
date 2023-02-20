""" This code helps to configure, calibrate and use an Odrive with a TSDZ2 motor."""

import time
import json
import threading

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
    AXIS_ERROR_NONE,
    CONTROLLER_ERROR_NONE,
    ENCODER_ERROR_NONE,
    MOTOR_ERROR_NONE,
    SENSORLESS_ESTIMATOR_ERROR_NONE,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED,
)
from enums import ControlMode

# Path of the configuration settings
gains_path = "gains.json"
hardware_and_security_path = "hardware_and_security.json"


class OdriveEncoderHall:
    """
    Represents a motor controlled by an Odrive with the integrated Hall encoder. This script has been written for one
    Odrive and one motor TSDZ2 wired on the axis0 of the Odrive. If a motor happens to be wired on axis1 the all the
    occurrences of `axis0` would need to be set to `axis1`. If the user wants to connect a second Odrive, the script
    would need to be adapted with `odrv1`.
    """

    def __init__(self, enable_watchdog=True, watchdog_timeout: float = 0.1, watchdog_feed_time: float = 0.01):
        self._watchdog_is_ready = False
        self._watchdog_timeout = watchdog_timeout
        self._watchdog_feed_time = watchdog_feed_time
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        print("Odrive found")
        self._config_watchdog(enable_watchdog)

        self._control_mode = ControlMode.STOP
        self._relative_pos = 0

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

    def _config_watchdog(self, enable_watchdog: bool):
        """
        Configures a watchdog. If the odrive does not receive a watchdog before the time given in watchdog_timeout, it
        will disconnect.

        Parameters
        ----------
        enable_watchdog: bool
            Indicates if the user wants to enable the watchdog (True) or not (False)
        """
        self.odrv0.axis0.config.enable_watchdog = False
        if self.odrv0.axis0.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED:
            self.odrv0.axis0.error = AXIS_ERROR_NONE
        if enable_watchdog:
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
            self._watchdog_is_ready = True
            print("Watchdog enabled")

    def _watchdog_feed(self):
        """
        Feeds the watchdog. To be called by a daemon thread.
        """
        while True:
            self.odrv0.axis0.watchdog_feed()
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
            manually the gains (True)
        pos_gain: float
        k_vel_gain: float
        k_vel_integrator_gain: float
        current_gain: float
        current_integrator_gain: float
        bandwidth: float
        """
        if not custom:
            with open(gains_path, "r") as gain_file:
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
        with open(hardware_and_security_path, "r") as hardware_and_security_file:
            hardware_and_security = json.load(hardware_and_security_file)

        self.odrv0.config.enable_dc_bus_overvoltage_ramp = hardware_and_security["enable_dc_bus_overvoltage_ramp"]
        self.odrv0.config.dc_bus_overvoltage_ramp_start = hardware_and_security["dc_bus_overvoltage_ramp_start"]
        self.odrv0.config.dc_bus_overvoltage_ramp_end = hardware_and_security["dc_bus_overvoltage_ramp_end"]
        self.odrv0.config.gpio9_mode = hardware_and_security["gpio9_mode"]
        self.odrv0.config.gpio10_mode = hardware_and_security["gpio10_mode"]
        self.odrv0.config.gpio11_mode = hardware_and_security["gpio11_mode"]
        self.odrv0.config.enable_brake_resistor = hardware_and_security["enable_brake_resistor"]
        self.odrv0.config.brake_resistance = hardware_and_security["brake_resistance"]

        # Controller
        self.odrv0.axis0.controller.config.vel_limit = hardware_and_security["vel_limit"]
        self.odrv0.axis0.controller.config.vel_limit_tolerance = hardware_and_security["vel_limit_tolerance"]

        # Encoder
        self.odrv0.axis0.encoder.config.mode = hardware_and_security["mode"]  # Mode of the encoder
        self.odrv0.axis0.encoder.config.cpr = hardware_and_security["cpr"]  # Count Per Revolution
        self.odrv0.axis0.encoder.config.calib_scan_distance = hardware_and_security["calib_scan_distance"]

        # Motor
        self.odrv0.axis0.motor.config.motor_type = hardware_and_security["motor_type"]
        self.odrv0.axis0.motor.config.pole_pairs = hardware_and_security["pole_pairs"]
        self.odrv0.axis0.motor.config.torque_constant = hardware_and_security["torque_constant"]
        self.odrv0.axis0.motor.config.calibration_current = hardware_and_security["calibration_current"]
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = hardware_and_security[
            "resistance_calib_max_voltage"
        ]
        self.odrv0.axis0.motor.config.requested_current_range = hardware_and_security["requested_current_range"]
        self.odrv0.axis0.motor.config.current_control_bandwidth = hardware_and_security["current_control_bandwidth"]
        self.odrv0.axis0.motor.config.current_lim = hardware_and_security["current_lim"]
        self.odrv0.axis0.motor.config.torque_lim = hardware_and_security["torque_lim"]

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
            self._relative_pos = int(self.odrv0.axis0.encoder.pos_estimate)

        self.odrv0.axis0.controller.input_pos = (self._relative_pos + angle / 360)

        if self._control_mode != ControlMode.POSITION_CONTROL:
            # Starts the motor if the previous control mode was not already `POSITION_CONTROL`
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self._control_mode = ControlMode.POSITION_CONTROL

    def velocity_control(self, velocity: float = 0.0, ramp_rate: float = 0.5):
        """
        Sets the motor to a given velocity in turn/s with velocities ramped at each change.

        Parameters
        ----------
        velocity: float
            Targeted velocity in turn/s.
        ramp_rate: float
            Ramp rate in turn/(s^2).
        """
        if velocity > self.odrv0.axis0.controller.config.vel_limit:
            raise ValueError(
                f"The velocity limit is {self.odrv0.axis0.controller.config.vel_limit} tr/s."
                f"Velocity specified: {velocity} tr/s"
            )

        if self._control_mode != ControlMode.VELOCITY_CONTROL:
            self.stop()
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.odrv0.axis0.controller.config.vel_ramp_rate = ramp_rate

        self.odrv0.axis0.controller.input_vel = velocity

        if self._control_mode != ControlMode.VELOCITY_CONTROL:
            # Starts the motor if the previous control mode was not already `VELOCITY_CONTROL`
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self._control_mode = ControlMode.VELOCITY_CONTROL

    def set_torque(self, torque: float):
        """
        Set the odrive in torque control, choose the torque and start the motor.
        torque: float
            Torque that the motor will produce
        """
        # TODO: Once the mechanical system is robust remove the abs and the "-"
        if torque > self.odrv0.axis0.motor.config.torque_lim:
            raise ValueError(
                "Error : Torque max ", self.odrv0.axis0.motor.config.torque_lim, ". torque given : ", torque
            )
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.odrv0.axis0.controller.input_torque = -abs(torque)
        # Starts the motor
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def set_torque_2(self, torque_goal: float, speed: float):
        """ """
        self.odrv0.axis0.controller.config.vel_limit = speed
        self.odrv0.axis0.controller.config.vel_limit_tolerance = 80 - speed
        self.odrv0.axis0.controller.input_torque = 0.1
        actual_torque = 0.1
        # Starts the motor
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        while True and actual_torque != torque_goal:
            print("Actual torque :", actual_torque)
            time.sleep(2)
            if abs(actual_torque - torque_goal) > 0.02 and actual_torque > torque_goal:
                actual_torque = self.odrv0.axis0.controller.input_torque = actual_torque - 0.02
            elif abs(actual_torque - torque_goal) > 0.02 and actual_torque < torque_goal:
                actual_torque = self.odrv0.axis0.controller.input_torque = actual_torque + 0.02
            else:
                actual_torque = self.odrv0.axis0.controller.input_torque = torque_goal

        print("Torque reached")

        stop = False
        while True and not stop:
            print("Actual torque :", actual_torque)
            up_or_down = input("Up (u) or Down (d) by 0.02 or Stop (s) : ")
            if up_or_down == "u":
                actual_torque = self.odrv0.axis0.controller.input_torque = actual_torque + 0.02
            elif up_or_down == "d":
                actual_torque = self.odrv0.axis0.controller.input_torque = actual_torque - 0.02
            elif up_or_down == "s":
                stop = True
                # Stops the motor
                self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            else:
                print("Command not supported")

    def stop(self, vel_stop: float = 0.5, ramp_rate: float = 0.5):
        """
        Stops the motor gently.

        Parameters
        ----------
        vel_stop: float
            The velocity at which the motor will be stopped if it was turning.
        ramp_rate: float
            The ramp_rate of the deceleration.
        """
        if abs(self.odrv0.axis0.encoder.vel_estimate) > vel_stop:
            # Gently slows down the motor.
            if self._control_mode != ControlMode.VELOCITY_CONTROL:
                self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
                self.odrv0.axis0.controller.config.vel_ramp_rate = ramp_rate

            self.odrv0.axis0.controller.input_vel = 0.0

            if self._control_mode != ControlMode.VELOCITY_CONTROL:
                self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            while abs(self.odrv0.axis0.encoder.vel_estimate) > vel_stop:
                pass

        # Stops the motor
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis0.controller.input_torque = 0.0
        self._control_mode = ControlMode.STOP

    def get_angle_motor(self) -> float:
        """
        Gives the angles.
        We consider the initial position to be 0° as a result a calibration is needed.
        """
        return self.odrv0.axis0.encoder.pos_estimate - self._relative_pos

    def get_estimated_velocity(self) -> float:
        """
        Gives the angles.
        We consider the initial position to be 0° as a result a calibration is needed.
        """
        return self.odrv0.axis0.encoder.vel_estimate  # vel_estimate is in turns/s

    def get_monitoring_commands(self):
        """ """
        return [
            self.odrv0.axis0.encoder.pos_estimate,
            self.odrv0.axis0.encoder.vel_estimate,
            self.odrv0.axis0.motor.current_control.Iq_setpoint,
            self.odrv0.axis0.motor.current_control.Iq_measured,
            self.odrv0.axis0.controller.mechanical_power,
            self.odrv0.axis0.controller.electrical_power,
        ]