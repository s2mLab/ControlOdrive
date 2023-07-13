"""
This file contains the computations that can be done with the motor data. With without any odrive connected.
"""
import json
import numpy as np

from pathlib import Path

json_path = Path(__file__).resolve().parent.parent / "parameters/hardware_and_security.json"


class MotorComputations:
    """
    This class contains the computations that can be done with the motor data. With or without any odrive connected.
    """

    def __init__(self, hardware_and_security_path: str = json_path):
        with open(hardware_and_security_path, "r") as hardware_and_security_file:
            self.hardware_and_security = json.load(hardware_and_security_file)

        self.reduction_ratio = self.hardware_and_security["reduction_ratio"]
        self.torque_constant = self.hardware_and_security["torque_constant"]

        self.resisting_current_proportional = self.hardware_and_security["resisting_current_proportional"]
        self.resisting_current_constant = self.hardware_and_security["resisting_current_constant"]
        self.prop_gear = self.hardware_and_security["resisting_current_prop_gear"]
        self.prop_cst = self.hardware_and_security["resisting_current_prop_constant"]
        self.cst_gear = self.hardware_and_security["resisting_current_cst_gear"]
        self.cst_cst = self.hardware_and_security["resisting_current_cst_constant"]

        self.gears = self.hardware_and_security["teeth"]

    def compute_resisting_current_coefficients(self, gear: int = 0):
        """
        Returns the coefficients of the linear function that gives the resisting current as a function of the velocity.

        Parameters
        ----------
        gear: int
            The current gear (0 if the chain is not on the motor, 1 for the easiest gear 10 for the hardest gear).

        Returns
        -------
        resisting_current_proportional, resisting_current_constant: float, float
        """
        if gear == 0:
            return (
                self.hardware_and_security["resisting_current_proportional"],
                self.hardware_and_security["resisting_current_constant"],
            )
        else:
            resisting_current_proportional = (
                self.prop_gear / self.gears[gear] + self.prop_cst
            )
            resisting_current_constant = (
                self.cst_gear / self.gears[gear] + self.cst_cst
            )
            return resisting_current_proportional, resisting_current_constant

    @staticmethod
    def compute_angle(turns: float) -> float:
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

    def compute_cadence(self, vel_estimate: float) -> float:
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
        return -vel_estimate * self.reduction_ratio * 60

    def compute_resisting_current_for_positive_velocity(self, vel_estimate: float, gear: int = 0) -> float:
        """
        Returns the current corresponding to the resisting torque.

        Parameters
        ----------
        vel_estimate: float
            The estimated velocity of the motor in turn/s.
        gear: int
            The current gear (0 if the chain is not on the motor, 1 for the easiest gear 10 for the hardest gear).


        Returns
        -------

        """
        resisting_current_proportional, resisting_current_constant = self.compute_resisting_current_coefficients(gear)
        return np.sign(vel_estimate) * (resisting_current_proportional * abs(vel_estimate) + resisting_current_constant)

    def compute_resisting_current(
        self, i_measured: float, vel_estimate: float, gear: int = 0
    ) -> float:
        """
        Returns the current corresponding to the resisting torque.

        Parameters
        ----------
        i_measured : float
            The measured current in A.
        vel_estimate : float
            The estimated velocity of the motor in turn/s.
        gear: int
            The current gear (0 if the chain is not on the motor, 1 for the easiest gear 10 for the hardest gear).

        Returns
        -------
        resisting_current : float
            The current corresponding to the resisting torque.
        """
        _, resisting_current_constant = self.compute_resisting_current_coefficients(gear)
        if vel_estimate != 0.0:
            resisting_current = self.compute_resisting_current_for_positive_velocity(vel_estimate, gear)
        else:
            # As the motor is not moving, we consider that all the current under the resisting_current_constant is
            # dissipated in the motor, the rest corresponds to the user torque. This is not what actually happens but
            # this choice has been made, in case of the study of a static movement it has to be adapted.
            resisting_current = -np.sign(i_measured) * min(resisting_current_constant, abs(i_measured))
        return resisting_current

    def compute_resisting_torque(
        self, i_measured: float, vel_estimate: float, gear: int = 0
    ) -> float:
        """
        Returns the resisting torque.

        Parameters
        ----------
        i_measured : float
            The measured current in A.
        vel_estimate : float
            The estimated velocity of the motor in turn/s. `vel_estimate` is negative if pedaling forward, positive if
            pedaling backward.
        gear: int
            The current gear (0 if the chain is not on the motor, 1 for the easiest gear 10 for the hardest gear).

        Returns
        -------
        resisting_torque : float
            The resisting torque due to solid frictions in Nm at the pedals.
        """
        return (
            self.torque_constant
            * self.compute_resisting_current(i_measured, vel_estimate, gear)
            / self.reduction_ratio
        )

    def compute_user_torque(self, i_measured: float, vel_estimate: float, gear: int = 0) -> float:
        """
        Returns the measured user torque (the resisting torque is subtracted from the motor torque).

        Parameters
        ----------
        i_measured : float
            The measured current in A.
        vel_estimate : float
            The estimated velocity of the motor in turn/s at the pedals.
        gear: int
            The current gear (0 if the chain is not on the motor, 1 for the easiest gear 10 for the hardest gear).

        Returns
        -------
        user_torque : float
            The measured user torque in Nm at the pedals.
        """
        return (
            -self.compute_resisting_torque(i_measured, vel_estimate, gear)
            - self.torque_constant * i_measured / self.reduction_ratio
        )

    def compute_motor_torque(self, i_measured: float) -> float:
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
        return self.torque_constant * i_measured / self.reduction_ratio

    @staticmethod
    def compute_user_power(user_torque: float, cadence: float) -> float:
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
