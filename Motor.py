""" This code helps to configure, calibrate and use an odrive with a TSDZ2 motor."""

import time

import odrive
from odrive.enums import *


class OdriveEncoderHall:
    """
    Represents a motor controlled by an odrive with the integrated Hall encoder.
    """

    def __init__(self):
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        self.velocity_limit = 5
        self.torque_limit = 0
        print("Odrive found")
        self._mode = EncoderMode.HALL
        self._cpr = 6 * 8
        self._calib_scan_distance = 150
        self._bandwidth = 100
        self._pole_pairs = 8
        self._vel_limit = 20
        self._shadow_count_init = self.odrv0.axis0.encoder.shadow_count
        self._angle_motor = 0
        self._angle_crank = 0
        self._old_shadow = 0

    def erase_configuration(self):
        """
        Erases the configuration present on the odrive.
        Error "[LEGACY_OBJ] protocol failed with 3 - propagating error to application" is not an issue.
        """

        try:
            self.odrv0.erase_configuration()
        except:
            pass
        self.odrv0 = odrive.find_any()
        try:
            self.odrv0.reboot()
        except:
            pass
        self.odrv0 = odrive.find_any()

    def save_configuration(self):
        """
        Saves the configuration present on the odrive.
        Error "[LEGACY_OBJ] protocol failed with 3 - propagating error to application" is not an issue.
        """

        try:
            self.odrv0.save_configuration()
        except:
            pass
        self.odrv0 = odrive.find_any()
        try:
            self.odrv0.reboot()
        except:
            pass
        self.odrv0 = odrive.find_any()

    def _config_encoder(
        self,
        mode: int,
        cpr: int,
        bandwidth: int,
        calib_scan_distance: int = None,
    ):
        """
        Configures the Encoder.
        Parameters
        ----------
        mode: int
            Correspond to the encoder mode (EncoderMode. )
        cpr: int
            Count per revolution of the encoder.
        bandwidth: int
            Bandwidth of the encoder.
        calib_scan_distance: int
            Distance of the calibration.
        """

        self.odrv0.axis0.encoder.config.mode = mode  # Mode of the encoder
        self.odrv0.axis0.encoder.config.cpr = cpr  # Count Per Revolution
        self.odrv0.axis0.encoder.config.bandwidth = bandwidth
        self.odrv0.axis0.encoder.config.calib_scan_distance = calib_scan_distance

        self.odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
        self.odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
        self.odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL

    def _config_motor(self, pole_pairs: int):
        """
        Configures the motor.
        pole_pairs: int
            Number of pole pairs (pairs of permanent magnet) in the motor.
        """
        torque_limit = 10

        self.odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.odrv0.axis0.motor.config.pole_pairs = pole_pairs

        self.odrv0.axis0.motor.config.calibration_current = 10
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = 20
        self.odrv0.axis0.motor.config.requested_current_range = 25  # Requires config save and reboot
        self.odrv0.axis0.motor.config.current_control_bandwidth = 100
        self.odrv0.axis0.motor.config.torque_constant = 0.21  # Not sure of this value

        self.odrv0.axis0.motor.config.current_lim = 20

        # By default torque limit is inf
        # self.odrv0.axis0.motor.config.torque_lim = torque_limit
        self.torque_limit = torque_limit

    def _config_brake_resistor(self):
        """
        Configures the brake resistor.
        """

        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.brake_resistance = 3.3

    def _config_overvoltage(self):
        """
        Configures the monitoring of over voltage.
        """
        self.odrv0.config.enable_dc_bus_overvoltage_ramp = True
        self.odrv0.config.dc_bus_overvoltage_ramp_start = 48.2
        self.odrv0.config.dc_bus_overvoltage_ramp_end = 48.35

    def _config_controller(self, vel_limit: float):
        """
        Configures the controller.
        vel_limit: float
            Velocity limit of the motor.
        """

        self.odrv0.axis0.controller.config.pos_gain = 1  # For position control
        self.odrv0.axis0.controller.config.vel_gain = (
            0.02 * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.encoder.config.cpr
        )
        self.odrv0.axis0.controller.config.vel_integrator_gain = (
            0.1 * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.encoder.config.cpr
        )
        self.odrv0.axis0.controller.config.vel_limit = vel_limit
        self.velocity_limit = vel_limit

        self.odrv0.axis0.controller.config.vel_limit_tolerance = 2

    def calibration(self):
        """
        Calibrates the odrive.
        """
        print("Start motor calibration")
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(38)

        if (
            self.odrv0.error == 0
            and self.odrv0.axis0.error == 0
            and self.odrv0.axis0.motor.error == 0
            and self.odrv0.axis0.error == 0
            and self.odrv0.axis0.controller.error == 0
            and self.odrv0.axis0.sensorless_estimator.error == 0
        ):
            self.confirm_configuration_calibration()
            self.save_configuration()
            print("Calibration done")
        else:
            raise RuntimeError("Error with configuration and/or calibration. Check odrivetool -> dump_errors(odr0)")

    def configuration(self):
        """
        Configures the odrive.
        """
        print("Configuration for HALL encoder")
        self._config_encoder(self._mode, self._cpr, self._bandwidth, calib_scan_distance=self._calib_scan_distance)
        self._config_motor(self._pole_pairs)
        self._config_brake_resistor()
        self._config_overvoltage()
        self._config_controller(self._vel_limit)
        self.save_configuration()

        print("Configuration done")

    def confirm_configuration_calibration(self):
        """
        Confirms the configuration and calibration. Allows to save the configuration after reboot.
        """
        self.odrv0.axis0.encoder.config.pre_calibrated = True
        self.odrv0.axis0.motor.config.pre_calibrated = True

    def position_control(self, angle: float = 0.0):
        """
        This function must be used only once at the beginning of a velocity control.

        Parameters
        ----------
        angle: float
            The angle the motor must go to (deg)
        """
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_pos = angle / 360  # `input_pos` is in turns

    def set_position(self, angle):
        self.odrv0.axis0.controller.input_pos = angle / 360  # `input_pos` is in turns

    def velocity_control(self, velocity: float = 0.0, ramp_rate: float = 0.5):
        """
        Sets the motor to a given velocity in turn/s with velocities ramped at each change.
        velocity: float
            Targeted velocity in (tr/s).
        """
        if velocity > self.velocity_limit:
            raise ValueError("Error : Velocity max ", self.velocity_limit, ". velocity given : ", turn_s)
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.vel_ramp_rate = ramp_rate
        self.odrv0.axis0.controller.input_vel = velocity

    def set_velocity(self, speed: float):
        self.odrv0.axis0.controller.input_vel = speed

    def set_torque(self, torque: float):
        """
        Set the odrive in torque control, choose the torque and start the motor.
        torque: float
            Torque that the motor will produce
        """
        # TODO: Once the mechanical system is robust remove the abs and the "-"
        if torque > self.torque_limit:
            raise ValueError("Error : Torque max ", self.torque_limit, ". torque given : ", torque)
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.odrv0.axis0.controller.input_torque = -abs(torque)
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def set_torque_2(self, torque_goal: float, speed: float):
        self.odrv0.axis0.controller.config.vel_limit = speed
        self.odrv0.axis0.controller.config.vel_limit_tolerance = 80 - speed
        self.odrv0.axis0.controller.input_torque = 0.1
        actual_torque = 0.1
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        while 1 and actual_torque != torque_goal:
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
        while 1 and not stop:
            print("Actual torque :", actual_torque)
            up_or_down = input("Up (u) or Down (d) by 0.02 or Stop (s) : ")
            if up_or_down == "u":
                actual_torque = self.odrv0.axis0.controller.input_torque = actual_torque + 0.02
            elif up_or_down == "d":
                actual_torque = self.odrv0.axis0.controller.input_torque = actual_torque - 0.02
            elif up_or_down == "s":
                stop = True
                self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            else:
                print("Command not supported")

    def stop(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis0.controller.input_torque = 0

    def get_angle_motor(self) -> float:
        """
        Gives the angles.
        We consider the initial position to be 0° as a result a calibration is needed.

        Returns
        -------
        pos_estimate : float
            Motor angle value (deg)
        """
        return self.odrv0.axis0.encoder.pos_estimate

    def get_estimated_velocity(self) -> float:
        """
        Gives the angles.
        We consider the initial position to be 0° as a result a calibration is needed.

        Returns
        -------
        pos_estimate : float
            Motor angle value (deg)
        """
        return self.odrv0.axis0.encoder.vel_estimate  # vel_estimate is in turns/s

    def get_monitoring_commands(self):
        return [
            self.odrv0.axis0.encoder.pos_estimate,
            self.odrv0.axis0.encoder.vel_estimate,
            self.odrv0.axis0.motor.current_control.Iq_setpoint,
            self.odrv0.axis0.motor.current_control.Iq_measured,
            self.odrv0.axis0.controller.mechanical_power,
            self.odrv0.axis0.controller.electrical_power,
        ]