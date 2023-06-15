"""
This script is used to sample the current corresponding to the resisting torque of the motor at different velocities.
"""
import json
import matplotlib.pyplot as plt
import numpy as np
import time

from scipy.optimize import curve_fit

from ergocycleS2M.motor_control.enums import DirectionMode
from ergocycleS2M.motor_control.motor import MotorController

# Initialisation
motor = MotorController()
motor.set_direction(DirectionMode.REVERSE)


def calibration(instruction, nb_turns=5):
    """
    This function is used to sample the current corresponding to the resisting torque of the motor at different
    cadences.

    Parameters
    ----------
    instruction
        The cadence instruction in rpm.
    nb_turns
        The number of turns to sample the data at the desired cadence.
    """
    if instruction > 0:
        motor.set_direction(DirectionMode.FORWARD)
    else:
        motor.set_direction(DirectionMode.REVERSE)
    motor.cadence_control(instruction)

    print(f"Waiting to stabilize at {instruction} rpm...")

    # Wait for to reach the instruction
    while not (instruction - 1 < motor.get_cadence() < instruction + 1):
        pass

    t0 = time.time()
    t1 = time.time()

    # Wait for 1 second to stabilize at the instructed velocity (can't be done with a time.sleep() because of the
    # watchdog thread)
    while t1 - t0 < 1.0:
        t1 = time.time()

    print(f"Stabilized at {instruction} rpm.")

    t0 = time.time()
    t1 = time.time()

    # Save the data as frequently as possible during nb_turns
    iq_measured = []
    vel_estimate = []
    while t1 - t0 < nb_turns / (abs(instruction) / 60):
        t1 = time.time()
        iq_measured.append(motor.axis.motor.current_control.Iq_measured)
        vel_estimate.append(motor.axis.encoder.vel_estimate)

    return np.mean(iq_measured), np.mean(vel_estimate)


if __name__ == "__main__":
    intensities = []
    velocities = []
    for ins in range(-60, 61, 5):
        if ins == 0:
            average_iq_measured, average_vel_estimate = calibration(-1, 1)
            intensities.append(average_iq_measured)
            velocities.append(average_vel_estimate)
            average_iq_measured, average_vel_estimate = calibration(1, 1)
            intensities.append(average_iq_measured)
            velocities.append(average_vel_estimate)
        else:
            average_iq_measured, average_vel_estimate = calibration(ins)
            intensities.append(average_iq_measured)
            velocities.append(average_vel_estimate)

    def lost_current(
            vel_estimate, resisting_current_proportional, resisting_current_constant
    ):
        """
        The current corresponding to the resisting torque of the motor is modeled as a linear function of the velocity.

        Parameters
        ----------
        vel_estimate:
            The velocity of the motor.
        resisting_current_proportional:
            The proportional coefficient.
        resisting_current_constant:
            The constant coefficient.

        Returns
        -------

        """
        return np.sign(vel_estimate) * (
                resisting_current_proportional * np.abs(vel_estimate) + resisting_current_constant
        )


    popt = curve_fit(lost_current, np.asarray(velocities), np.asarray(intensities), p0=[0.01, 0.5])

    motor.stop()

    plt.plot(velocities, intensities)
    plt.plot(velocities, lost_current(velocities, *popt[0]))
    plt.xlabel("Motor velocity (tr/s)")
    plt.ylabel("Resisting current (A)")
    plt.show()

    with open("./ergocycleS2M/parameters/hardware_and_security.json", "r") as f:
        hardware_and_security = json.load(f)

    hardware_and_security["resisting_current_proportional"] = - float(popt[0][0])
    hardware_and_security["resisting_current_constant"] = - float(popt[0][1])

    # Writing to .json
    json_object = json.dumps(hardware_and_security, indent=4)
    with open("./ergocycleS2M/parameters/hardware_and_security.json", "w") as outfile:
        outfile.write(json_object)