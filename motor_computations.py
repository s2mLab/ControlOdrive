"""
This file contains all the computations that can be done without any odrive connected.
"""

import json
import numpy as np


class MotorComputations:
    """
    This class contains all the computations that can be done without any odrive connected.
    """

    def __init__(self, hardware_and_security_path: str = "parameters/hardware_and_security.json"):
        with open(hardware_and_security_path, "r") as hardware_and_security_file:
            self.hardware_and_security = json.load(hardware_and_security_file)

        self._reduction_ratio = self.hardware_and_security["reduction_ratio"]

        self.resisting_current_coeff_proportional = self.hardware_and_security["resisting_current_coeff_proportional"]
        self.resisting_current_coeff_power = self.hardware_and_security["resisting_current_coeff_power"]

    def compute_cadence(self, vel_estimate: float):
        """
        Returns the estimated cadence of the pedals in rpm. Can be called without any odrive connected.
        """
        return - vel_estimate * self._reduction_ratio * 60

    @staticmethod
    def compute_angle(turns: float):
        """
        Returns the estimated angle in degrees. Can be called without any odrive connected.
        """
        return (turns * 360) % 360

    @staticmethod
    def compute_user_power(user_torque, cadence):
        """
        Returns the user power in W.
        """
        return user_torque * cadence * 2 * np.pi / 60

    def compute_resisting_torque(self, i_measured, vel_estimate):
        """
        Returns the resisting torque.
        """
        if vel_estimate != 0.0:
            dyn_resisting_current = - self.resisting_current_coeff_proportional * vel_estimate / abs(vel_estimate) \
                                    * abs(vel_estimate) ** (1 / self.resisting_current_coeff_power)
        else:
            if abs(i_measured) <= self.hardware_and_security["resisting_torque_current"]:
                dyn_resisting_current = i_measured
            else:
                dyn_resisting_current = self.hardware_and_security["resisting_torque_current"]
        return - self.hardware_and_security["torque_constant"] * dyn_resisting_current / self._reduction_ratio

    def compute_user_torque(
            self,
            i_measured,
            vel_estimate,
    ):
        """
        Returns the measured user torque (the resisting torque is subtracted from the motor_torque). Can be called
        without any odrive connected.
        """
        if vel_estimate != 0.0:
            dyn_resisting_current = - self.resisting_current_coeff_proportional * vel_estimate / abs(vel_estimate) \
                                    * abs(vel_estimate) ** (1 / self.resisting_current_coeff_power)
            i_user = i_measured - dyn_resisting_current
        else:
            if abs(i_measured) <= self.hardware_and_security["resisting_torque_current"]:
                i_user = 0.0
            else:
                i_user = i_measured
        return - self.hardware_and_security["torque_constant"] * i_user / self._reduction_ratio
