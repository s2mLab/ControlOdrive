"""
This script was used to compute the resisting torque of the Ergocycle and the corresponding current in case the
torque constant was recalculated.
"""
import json
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit


def aff_cst(t, a, break_time):
    """
    Defines a function that is a ramp a * t for t < T and constant after.

    Parameters
    ----------
    t: list | np.ndarray
        Time.
    a: float
        Ramp rate.
    break_time: float
        Time from which the function is constant.
    """
    np_break_time = break_time * np.ones(len(t))
    return a * np.amin([t, np_break_time], axis=0)


resisting_torques = []
resisting_currents = []

# The .json files have been obtained with the following values, they were used to recalculate the current
# corresponding to the resisting torque.
torque_constant_the_day_of_the_XP = 0.0822558306452444
reduction_ratio = 8 / 36 * 10 / 91

for mode in ("Concentric", "Eccentric"):
    for ramp in (5, 10):
        for i in range(1, 6):
            with open(f"find_resisting_torque_{mode}_2_{ramp}_{i}.json", "r") as file:
                monitoring_commands = json.load(file)

            time = np.asarray(monitoring_commands["time"])
            torque = np.asarray(monitoring_commands["Iq_measured"])
            vel_estimate = np.asarray(monitoring_commands["vel_estimate"])
            last_zero = time[np.where(vel_estimate == 0)[0][-1]]

            popt, pcov = curve_fit(aff_cst, time, torque, p0=[-0.5, 3.001])

            plt.plot(time, aff_cst(time, *popt))
            plt.plot(time, torque)
            plt.show()

            resisting_torques.append(abs(aff_cst([last_zero], *popt)))
            resisting_currents.append(
                abs(aff_cst([last_zero], *popt)) / torque_constant_the_day_of_the_XP * reduction_ratio
            )
            # print(mode, ramp, resisting_torques[-1])

print(f"Resisting torque: {np.mean(resisting_torques)} (+/- {np.std(resisting_torques)}) Nm")
print(f"Resisting current: {np.mean(resisting_currents)} (+/- {np.std(resisting_currents)}) A")

# Resisting torque: 1.403089921584192 (+/- 0.31551030723591406) Nm
# Resisting current: 0.41654785903687247 (+/- 0.09366836790816456) A