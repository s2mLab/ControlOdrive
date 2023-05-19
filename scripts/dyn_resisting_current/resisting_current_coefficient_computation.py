"""
This script is used to calculate the coefficients of the current corresponding to the resisting torque of the motor.
"""
import json
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit

with open("./parameters/hardware_and_security.json", "r") as hardware_and_security_file:
    hardware_and_security = json.load(hardware_and_security_file)

with open("./dyn_resisting_current/velocity_and_current_from_dyn_calibration.json", "r") as file:
    i_v = json.load(file)

resisting_current = np.asarray(i_v["resisting_current"])
motor_velocity = np.asarray(i_v["motor_velocity"]) / 60 / hardware_and_security["reduction_ratio"]


def lost_current(
    x,
    resisting_current_coeff_proportional,
    resisting_current_coeff_power,
    resisting_current_coeff_offset
):
    return - resisting_current_coeff_proportional * x / abs(x) * abs(x)**(1/resisting_current_coeff_power)\
        + resisting_current_coeff_offset


popt, _ = curve_fit(lost_current, motor_velocity, resisting_current, p0=[1, 3, 0])

print(popt)

hardware_and_security["resisting_current_coeff_proportional"] = popt[0]
hardware_and_security["resisting_current_coeff_power"] = popt[1]

# Writing to .json
json_object = json.dumps(hardware_and_security, indent=4)
with open("./parameters/hardware_and_security.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(motor_velocity, resisting_current)
plt.plot(motor_velocity, lost_current(motor_velocity, *popt))

plt.xlabel("Motor velocity (tr/s)")
plt.ylabel("Resisting current (A)")

plt.show()
