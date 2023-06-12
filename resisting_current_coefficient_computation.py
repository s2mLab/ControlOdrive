"""
This script is used to calculate the coefficients of the current corresponding to the resisting torque of the motor.
"""
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

from scipy.optimize import curve_fit

from ergocycleS2M.data_processing.load import load, read, compute_data

script_path = sys.argv[0]
control_odrive_directory = os.path.dirname(os.path.abspath(script_path))

data = load(control_odrive_directory + "/data_from_dyn_calibration_54.bio")
smoothed_data = read(control_odrive_directory + "/data_from_dyn_calibration_54.bio", 100, 100)

instruction = np.asarray(data["instruction"])
resisting_current = np.asarray(data["iq_measured"])
motor_velocity = np.asarray(data["vel_estimate"])
(
    _,
    data["cadence"],
    _, _, _,
    data["motor_torque"],
) = compute_data(data["vel_estimate"], data["turns"], data["iq_measured"])

i = []
v = []
std = []
cad = []
for ins in range(-60, 61, 5):
    if ins == 0:
        i.append(np.mean(resisting_current[instruction == -1]))
        v.append(np.mean(motor_velocity[instruction == -1]))
        std.append(np.std(data["motor_torque"][instruction == -1]))
        cad.append(np.mean(data["cadence"][instruction == -1]))
        i.append(np.mean(resisting_current[instruction == 1]))
        v.append(np.mean(motor_velocity[instruction == 1]))
        std.append(np.std(data["motor_torque"][instruction == 1]))
        cad.append(np.mean(data["cadence"][instruction == 1]))
    else:
        i.append(np.mean(resisting_current[instruction == ins]))
        v.append(np.mean(motor_velocity[instruction == ins]))
        std.append(np.std(data["motor_torque"][instruction == ins]))
        cad.append(np.mean(data["cadence"][instruction == ins]))

def lost_current(
        x, a, b
):
    return np.sign(x) * (a * np.abs(x) + b)


popt, _ = curve_fit(lost_current, v, i, p0=[0.01, 0.5])

print(popt)

# hardware_and_security["resisting_current_coeff_proportional"] = popt[0]
# hardware_and_security["resisting_current_coeff_power"] = popt[1]
#
# # Writing to .json
# json_object = json.dumps(hardware_and_security, indent=4)
# with open("./parameters/hardware_and_security.json", "w") as outfile:
#     outfile.write(json_object)

plt.plot(v, i)
plt.plot(v, lost_current(v, *popt))
plt.xlabel("Motor velocity (tr/s)")
plt.ylabel("Resisting current (A)")

plt.figure()

plt.plot(data["time"], resisting_current, label="Measured current")
plt.plot(smoothed_data["time_for_smoothed"], smoothed_data["smoothed_iq_measured"], label="Smoothed current")
plt.plot(data["time"], lost_current(motor_velocity, *popt), label="Approximated current according to the velocity")

plt.xlabel("Time (s)")
plt.ylabel("Resisting current (A)")
plt.legend()

plt.figure()
plt.plot(cad, std)
plt.xlabel("Cadence (rpm)")
plt.ylabel("Standard deviation of the torque (Nm)")
plt.title("Standard deviation of the torque according to the cadence in cadence control")
plt.show()