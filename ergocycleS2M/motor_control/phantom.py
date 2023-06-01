"""
This code is used to test the GUI mechanics without any ODrive connected. It may not be up-to-date.
"""
import time
import json
import copy
import numpy as np

from ergocycleS2M.data_processing.save import save
from ergocycleS2M.motor_control.enums import (
    ControlMode,
    control_modes_based_on_torque,
    control_modes_based_on_cadence,
    DirectionMode,
)
from ergocycleS2M.motor_control.motor_computations import MotorComputations


class Phantom(MotorComputations):
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
        gains_path: str = "parameters/gains.json",
        file_path: str = None,
    ):
        super(Phantom, self).__init__()
        self._watchdog_is_ready = False
        self._external_watchdog = external_watchdog
        self._watchdog_timeout = self.hardware_and_security["watchdog_timeout"]
        self._watchdog_feed_time = self.hardware_and_security["watchdog_feed_time"]
        print("Look for an odrive ...")
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
        pass

    def save_configuration(self):
        """
        Saves the current configuration to non-volatile memory and reboots the board.
        fibre.libfibre.ObjectLostError "[LEGACY_OBJ] protocol failed with 3 - propagating error to application" is not
        an issue.
        """
        pass

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
        if enable_watchdog:
            return True
        else:
            return False

    def _internal_watchdog_feed(self):
        """
        Feeds the watchdog. To be called by a daemon thread.
        """
        while True:
            time.sleep(self._watchdog_feed_time)

    def watchdog_feed(self):
        """
        Feeds the watchdog. To be called by the user.
        """
        pass

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
        print("Calibration done")

    def has_error(self):
        """
        Indicates if one or several errors has been detected (True) or not (False).
        """
        return False

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

        self.save_configuration()

    def hardware_and_security_configuration(self):
        """
        Configures the settings linked to the hardware or the security not supposed to be changed by the user.
        """
        pass

    def get_sign(self):
        """
        Set the sign of the rotation depending on the rotation direction (reverse or forward).
        """
        if self._direction == DirectionMode.FORWARD:
            return 1
        else:
            return -1

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
        pass

    def turns_control(self, turns: float = 0.0):
        """
        Makes the motors turn for the indicated number of turns.

        Parameters
        ----------
        turns: float
            The number of turns the motor will do.
        """
        pass

    def zero_position_calibration(self):
        """
        Calibration for the 0 deg.
        """
        pass

    def position_control(self, angle: float = 0.0):
        """
        Leads the motors to the indicated angle.

        Parameters
        ----------
        angle: float
            The angle the motor must go to ([-180.0, 360.0] deg)
        """
        pass

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

        self._check_ramp_rate(cadence_ramp_rate)

        if self._control_mode not in control_modes_based_on_cadence:
            self.stopping()
            self.stopped()

        self._control_mode = control_mode

        return self.get_sign() * cadence

    def torque_control(
        self,
        user_torque: float = 0.0,
        torque_ramp_rate: float = 2.0,
        resisting_torque: float = None,
        control_mode: ControlMode = ControlMode.TORQUE_CONTROL,
    ):
        """
        Set the odrive in torque control, choose the torque and start the motor.

        Parameters
        ----------
        user_torque: float
            Torque of the user (Nm) at the pedals.
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
        vel_estimate = 0.0
        # `vel_estimate` is negative if pedaling forward, positive if pedaling backward.
        if (self._direction == DirectionMode.FORWARD and vel_estimate >= 0) or (
            self._direction == DirectionMode.REVERSE and vel_estimate <= 0
        ):
            input_motor_torque = motor_torque = 0.0
            torque_ramp_rate_motor = 100.0
        # If the user is pedaling, the torque and torque_ramp values have to be translated to the motor.
        else:
            # TODO: Check if the torque ramp rate is correct
            torque_ramp_rate_motor = torque_ramp_rate * self._reduction_ratio

        # The motor can be controlled with the computed values
        if self._control_mode not in control_modes_based_on_torque:
            self.stopping()
            self.stopped()

        # In case the previous control mode was based on torque control but was not `TORQUE_CONTROL`,
        # self._control_mode is updated.
        self._control_mode = control_mode

        return motor_torque  # Nm at the pedals

    def concentric_power_control(
        self, power: float = 0.0, torque_ramp_rate: float = 2.0, resisting_torque: float = None
    ):
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
        cadence = 0.0  # rad/s
        if cadence == 0:
            return self.torque_control(0.0, torque_ramp_rate, resisting_torque, ControlMode.CONCENTRIC_POWER_CONTROL)
        else:
            return self.torque_control(
                min(abs(power) / cadence, self.hardware_and_security["torque_lim"]),
                torque_ramp_rate,
                resisting_torque,
                ControlMode.CONCENTRIC_POWER_CONTROL,
            )

    def eccentric_power_control(self, power: float = 0.0, cadence_ramp_rate: float = 5.0, cadence_max: float = 50.0):
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
        if (self._direction == DirectionMode.REVERSE and torque >= 0) or (
            self._direction == DirectionMode.FORWARD and torque <= 0
        ):
            self.cadence_control(cadence_max, cadence_ramp_rate, ControlMode.ECCENTRIC_POWER_CONTROL)
            return np.inf  # So we know that the user is not forcing.
        else:
            cadence = min(abs(power / torque) / 2 / np.pi * 60, cadence_max)
            return self.cadence_control(cadence, cadence_ramp_rate, ControlMode.ECCENTRIC_POWER_CONTROL)

    def linear_control(self, linear_coeff: float = 0.0, torque_ramp_rate: float = 2.0, resisting_torque: float = None):
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

        self._control_mode = ControlMode.STOPPING

    def stopped(self):
        """
        Running until the motor is fully stopped. Can be executed in another thread or process.

        Returns
        -------
        True when the motor has stopped.
        """
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
        return 20 * np.sin(time.time() - self._relative_pos)

    def get_cadence(self) -> float:
        """
        Returns the estimated cadence of the pedals in rpm.
        """
        return self.compute_cadence(20 * np.sin(time.time()))

    def get_electrical_power(self):
        """
        Returns the electrical power in W.
        """
        return 20 * np.sin(time.time())

    def get_mechanical_power(self):
        """
        Returns the mechanical power in W.
        """
        return 20 * np.sin(time.time())

    def get_user_power(self):
        """
        Returns the user mechanical power in W.
        """
        return self.compute_user_torque(self.get_user_torque(), self.get_cadence())

    def get_iq_setpoint(self):
        """
        Returns the commanded motor current in A.
        """
        return 20 * np.sin(time.time())

    def get_iq_measured(self):
        """
        Returns the measured motor current in A.
        """
        return 20 * np.sin(time.time())

    def get_measured_torque(self):
        """
        Returns the measured torque.
        """
        return 20 * np.sin(time.time())

    def get_motor_torque(self):
        """
        Returns the measured torque.
        """
        return self.compute_motor_torque(20 * np.sin(time.time()))

    def get_resisting_torque(self):
        """
        Returns the resisting torque.
        """
        return self.compute_resisting_torque(
            20 * np.sin(time.time()),
            20 * np.sin(time.time()),
        )

    def get_user_torque(self):
        """
        Returns the measured user torque (the resisting torque is subtracted from the motor_torque).
        """
        return self.compute_user_torque(
            20 * np.sin(time.time()),
            20 * np.sin(time.time()),
        )

    def get_errors(self):
        """
        Returns the errors.
        """
        return ""

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
            "control_mode": self._control_mode.value,
            "direction": self._direction.value,
            "iq_setpoint": self.get_iq_setpoint(),
            "iq_measured": self.get_iq_measured(),
            "measured_torque": self.get_measured_torque(),
            "motor_torque": self.get_motor_torque(),
            "resisting_torque": self.get_resisting_torque(),
            "mechanical_power": self.get_mechanical_power(),
            "electrical_power": self.get_electrical_power(),
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
        training_mode: str = "",
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
            "control_mode": self._control_mode.value,
            "direction": self._direction.value,
            "training_mode": training_mode,
            "turns": self.get_turns(),
        }

        save(data, file_path)
