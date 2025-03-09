"""
This script allows to compute the coefficients of the linear model of the current corresponding to the resiting torque
as a function of the gear.
"""
import json
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit

with open("./ergocycleS2M/parameters/hardware_and_security.json", "r") as f:
    hardware_and_security = json.load(f)

gear2teeth = hardware_and_security["teeth"]
gears = [10, 8, 6, 4, 3, 2, 1]

resisting_current_proportional = []
resisting_current_constant = []

teeth = []

for gear in gears:
    with open(f"./calibration_files/resisting_current_gear_{gear}.json", "r") as f:
        data = json.load(f)
        resisting_current_proportional.append(data["a"])
        resisting_current_constant.append(data["b"])
        teeth.append(gear2teeth[gear])


def affine(x, prop, cst):
    return prop / x + cst


popt_constant = curve_fit(affine, teeth, resisting_current_constant)
popt_proportional = curve_fit(affine, teeth, resisting_current_proportional)

plt.plot(gears, resisting_current_constant, "o", label="Data")
plt.plot(gears, affine(teeth, *popt_constant[0]), label="Fitted line")
plt.xlabel("Gear")
plt.ylabel("Resisting current constant coefficients")
plt.legend()
plt.title("Resisting current constant coefficients as a function of the gear")

plt.figure()
plt.plot(gears, resisting_current_proportional, "o", label="Data")
plt.plot(gears, affine(teeth, *popt_proportional[0]), label="Fitted line")
plt.xlabel("Gear")
plt.ylabel("Resisting current constant coefficients")
plt.legend()
plt.title("Resisting current proportional coefficients as a function of the gear")

plt.show()

with open("./ergocycleS2M/parameters/hardware_and_security.json", "r") as f:
    hardware_and_security = json.load(f)

hardware_and_security["resisting_current_cst_gear"] = popt_constant[0][0]
hardware_and_security["resisting_current_cst_constant"] = popt_constant[0][1]
hardware_and_security["resisting_current_prop_gear"] = popt_proportional[0][0]
hardware_and_security["resisting_current_prop_constant"] = popt_proportional[0][1]

# Writing to .json
json_object = json.dumps(hardware_and_security, indent=4)
with open("./ergocycleS2M/parameters/hardware_and_security.json", "w") as outfile:
    outfile.write(json_object)
