"""
This file contains the computations that can be done with the motor data without any odrive connected.
"""

import json
import numpy as np


class MotorComputations:
    """
    This class contains the computations that can be done with the motor data without any odrive connected.
    """

    def __init__(self, hardware_and_security_path: str = "parameters/hardware_and_security.json"):
        with open(hardware_and_security_path, "r") as hardware_and_security_file:
            self.hardware_and_security = json.load(hardware_and_security_file)

        self._reduction_ratio = self.hardware_and_security["reduction_ratio"]
        self._torque_constant = self.hardware_and_security["torque_constant"]

        self.resisting_current_coeff_proportional = self.hardware_and_security["resisting_current_coeff_proportional"]
        self.resisting_current_coeff_power = self.hardware_and_security["resisting_current_coeff_power"]
        self._resisting_torque_current = self.hardware_and_security["resisting_torque_current"]

    @staticmethod
    def compute_angle(turns: float):
        """
        Returns the estimated angle in degrees.

        Parameters
        ----------
        turns : float
            The number of turns of the motor.

        Returns
        -------
        angle : float
            The estimated angle in degrees.
        """
        return (turns * 360) % 360

    def compute_cadence(self, vel_estimate: float):
        """
        Returns the estimated cadence of the pedals in rpm.

        Parameters
        ----------
        vel_estimate : float
            The estimated velocity of the motor in turn/s.

        Returns
        -------
        cadence : float
            The estimated cadence of the pedals in rpm.
        """
        return - vel_estimate * self._reduction_ratio * 60

    def compute_resisting_torque(self, i_measured, vel_estimate):
        """
        Returns the resisting torque.

        Parameters
        ----------
        i_measured : float
            The measured current in A.
        vel_estimate : float
            The estimated velocity of the motor in turn/s. `vel_estimate` is negative if pedaling forward, positive if
            pedaling backward.

        Returns
        -------
        resisting_torque : float
            The resisting torque due to solid frictions in Nm at the pedals.
        """
        if vel_estimate != 0.0:
            resisting_current = self.resisting_current_coeff_proportional * vel_estimate / abs(vel_estimate) \
                                    * abs(vel_estimate) ** (1 / self.resisting_current_coeff_power)
        else:
            # As the motor is not moving, all current is dissipated as heat and corresponds to the resisting current.
            resisting_current = i_measured
        return self._torque_constant * resisting_current / self._reduction_ratio

    def compute_user_torque(
            self,
            i_measured,
            vel_estimate,
    ):
        """
        Returns the measured user torque (the resisting torque is subtracted from the motor torque).

        Parameters
        ----------
        i_measured : float
            The measured current in A.
        vel_estimate : float
            The estimated velocity of the motor in turn/s at the pedals.

        Returns
        -------
        user_torque : float
            The measured user torque in Nm at the pedals.
        """
        return - self.compute_resisting_torque(i_measured, vel_estimate) - \
            self._torque_constant * i_measured / self._reduction_ratio

    def compute_motor_torque(self, i_measured):
        """
        Returns the measured motor torque.

        Parameters
        ----------
        i_measured : float
            The measured current in A.

        Returns
        -------
        motor_torque : float
            The measured motor torque in Nm at the pedals.
        """
        return self._torque_constant * i_measured / self._reduction_ratio

    @staticmethod
    def compute_user_power(user_torque, cadence):
        """
        Returns the user power in W.

        Parameters
        ----------
        user_torque : float
            The measured user torque in Nm at the pedals.
        cadence : float
            The estimated cadence of the pedals in rpm.

        Returns
        -------
        user_power : float
            The user power in W.
        """
        return user_torque * cadence * 2 * np.pi / 60
