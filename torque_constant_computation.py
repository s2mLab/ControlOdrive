"""
This script computes the torque constant of the motor using the calibration files. The torque constant allows to compute
the torque developed by the motor from the current measured at the motor terminals.
"""
import json
import matplotlib.pyplot as plt
import numpy as np

from scipy.optimize import curve_fit

from ergocycleS2M.data_processing.load import load, compute_data
from ergocycleS2M.motor_control.motor_computations import MotorComputations

motor_computations = MotorComputations()
reduction_ratio = motor_computations.reduction_ratio
previous_torque_constant = motor_computations.torque_constant

# Dimensions of the wrench used to apply the torque
wrench = 0.170

# Dictionary to store the data of the different calibrations
calibs = {
    "calib_1": {
        "file_name": "Calib1.bio",
        "mass": 11.70,
    },
    "calib_2": {
        "file_name": "Calib2.bio",
        "mass": 16.80,
    },
    "calib_3": {
        "file_name": "Calib3.bio",
        "mass": 8.80,
    },
    "calib_4": {
        "file_name": "Calib4.bio",
        "mass": 6.00,
    },
    "calib_5": {
        "file_name": "Calib5.bio",
        "mass": 3.15,
    },
}

for i in range(1, 6):
    calib_i = f"calib_{i}"
    data = load(calibs[calib_i]["file_name"])
    turns = np.asarray(data["turns"])
    # I forgot to reset the turns to 0 at the beginning of each calibration but this does the same since the wrench was
    # immobile and vertical at the beginning of each calibration
    turns = turns - turns[0]
    angle = motor_computations.compute_angle(turns)
    # The first 45 degrees to ignore the ramp up to the desired velocity
    i_start = np.where(angle > 45)[0][0]
    # We select the 2 first turns for each weight to have approximately the same number of points for each calibration
    i_end = np.where((angle > 45) & (turns > 2))[0][0]

    # Computing the torque at the motor from the weight and the angle
    angle = angle[i_start:i_end]
    weight = 9.81 * calibs[calib_i]["mass"]
    calibs[calib_i]["weight_torque_at_motor"] = weight * wrench * np.sin(np.deg2rad(angle)) * reduction_ratio

    # Computing the current at the motor from the measured current and the current corresponding to the resisting torque
    resisting_current = motor_computations.compute_resisting_torque_for_positive_velocity(
        np.asarray(data["vel_estimate"][i_start:i_end])
    )
    calibs[calib_i]["user_current"] = -np.asarray(data["iq_measured"])[i_start:i_end] - resisting_current

# Concatenate all the data to compute the torque constant with a linear regression
all_currents = np.concatenate([calibs[calib_i]["user_current"] for calib_i in calibs])
all_weight_torques = np.concatenate([-calibs[calib_i]["weight_torque_at_motor"] for calib_i in calibs])


def motor_torque(current: float | np.ndarray, trq_constant: float) -> float | np.ndarray:
    """
    Compute the torque at the motor from the current measured at the motor terminals.

    Parameters
    ----------
    current: float | np.ndarray
        All currents
    trq_constant: float
        Torque constant

    Returns
    -------
    torque_constant * current: float | np.ndarray
        The torque at the motor
    """
    return trq_constant * current


torque_constant = curve_fit(motor_torque, all_currents, all_weight_torques)[0][0]

r2 = 1 - np.sum((all_weight_torques - torque_constant * all_currents) ** 2) / np.sum(
    (all_weight_torques - np.mean(all_weight_torques)) ** 2
)
previous_r2 = 1 - np.sum((all_weight_torques - previous_torque_constant * all_currents) ** 2) / np.sum(
    (all_weight_torques - np.mean(all_weight_torques)) ** 2
)

print(
    f"The previous torque constant was {previous_torque_constant:.2f} Nm/A and its coefficient of determination with "
    f"this data was {previous_r2:.2f}. The new torque constant is {torque_constant:.2f} Nm/A and its coefficient "
    f"of determination with this data is {r2:.2f}."
)

with open("./ergocycleS2M/parameters/hardware_and_security.json", "r") as f:
    hardware_and_security = json.load(f)

hardware_and_security["torque_constant"] = torque_constant

# Writing to .json
json_object = json.dumps(hardware_and_security, indent=4)
with open("./ergocycleS2M/parameters/hardware_and_security.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(torque_constant * all_currents)
plt.plot(all_weight_torques)
plt.show()
