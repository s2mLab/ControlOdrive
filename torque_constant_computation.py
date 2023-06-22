"""

"""
import json
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path

from ergocycleS2M.data_processing.load import load, compute_data

hardware_and_security_path = Path(__file__).resolve().parent / "ergocycleS2M/parameters/hardware_and_security.json"

with open(hardware_and_security_path, "r") as hardware_and_security_file:
    hardware_and_security = json.load(hardware_and_security_file)
resisting_current_proportional = hardware_and_security["resisting_current_proportional"]
resisting_current_constant = hardware_and_security["resisting_current_constant"]

wrench = 0.170

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
    user_torque, _, angle, _, resisting_torque, motor_torque = compute_data(
        data["vel_estimate"],
        data["turns"],
        data["iq_measured"],
    )
    angle = (angle - angle[0]) % 360
    turns = turns - turns[0]
    calibs[calib_i]["i_start"] = i_start = np.where(angle > 45)[0][0]
    calibs[calib_i]["i_end"] = i_end = np.where((angle > 45) & (turns > 2))[0][0]
    calibs[calib_i]["angle"] = angle[i_start:i_end]
    calibs[calib_i]["iq_measured"] = np.asarray(data["iq_measured"])[i_start:i_end]
    weight = 9.81 * calibs[calib_i]["mass"]
    calibs[calib_i]["weight_torque"] = -weight * wrench * np.sin(np.deg2rad(calibs[calib_i]["angle"]))

    calibs[calib_i]["resisting_current"] = (np.sign(data["vel_estimate"]) * (
        resisting_current_proportional * np.abs(data["vel_estimate"]) + resisting_current_constant
    ))[i_start:i_end]

    plt.plot(calibs[calib_i]["resisting_current"])
    plt.plot(calibs[calib_i]["iq_measured"])
    plt.show()
