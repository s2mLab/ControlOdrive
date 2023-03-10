"""
This script was used to compute the current corresponding to the resisting torque of the Ergocycle.
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


resisting_currents = []

for mode in ("Concentric", "Eccentric"):
    for ramp in (5, 10):
        for i in range(1, 6):
            with open(f"find_resisting_torque/find_resisting_torque_{mode}_2_{ramp}_{i}.json", "r") as file:
                monitoring_commands = json.load(file)

            time = np.asarray(monitoring_commands["time"])
            iq_measured = np.asarray(monitoring_commands["Iq_measured"])
            vel_estimate = np.asarray(monitoring_commands["vel_estimate"])
            last_zero = time[np.where(vel_estimate == 0)[0][-1]]

            popt, pcov = curve_fit(aff_cst, time, iq_measured, p0=[-0.5, 3.001])

            # plt.plot(time, aff_cst(time, *popt))
            # plt.plot(time, iq_measured)
            # plt.show()

            resisting_currents.append(abs(aff_cst([last_zero], *popt)))
            # print(mode, ramp, resisting_currents[-1])

print(f"Resisting current: {np.mean(resisting_currents)} (+/- {np.std(resisting_currents)}) A")

# Resisting current: 0.42126813627221144 (+/- 0.057300774416392) A
