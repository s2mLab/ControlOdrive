"""
Calculates the coefficients to calculate the resisting current.
"""

import json
import numpy as np
from scipy.optimize import curve_fit
from scipy import signal
import matplotlib.pyplot as plt

with open("find_resisting_torque3.json", "r") as file:
    monitoring_commands = json.load(file)

fs = 50

time = np.asarray(monitoring_commands["time"])
iq_setpoint = np.asarray(monitoring_commands["iq_setpoint"])
vel_estimate = np.asarray(monitoring_commands["vel_estimate"])

b, a = signal.butter(2, 1, btype='lowpass', fs=fs)
iq_setpoint_filtered = signal.lfilter(b, a, iq_setpoint)

iq = []
v = []

for i in range(29):
    iq.append(np.mean(iq_setpoint_filtered[fs * 20 * i + fs * 5:fs * 20 * i + fs * 5 * 3]))
    v.append(np.mean(vel_estimate[fs * 20 * i + fs * 5:fs * 20 * i + fs * 5 * 3]))

iq = np.asarray(iq)
v = np.asarray(v)

iq = iq[v != 0]
v = v[v != 0]

iq_setpoint_filtered = iq_setpoint_filtered[vel_estimate != 0]
time = time[vel_estimate != 0]
vel_estimate = vel_estimate[vel_estimate != 0]


def res_i(x, c, d, e):
    """
    Doc
    """
    return - c * x / abs(x) * abs(x)**(1/e) + d


popt, pcov = curve_fit(res_i, v, iq, p0=[1, 0, 3])

print(popt)


# plt.plot(v, iq)
# plt.plot(v, res_i(v, *popt))
# plt.show()
#
#
plt.plot(time, iq_setpoint_filtered)
plt.plot(time, res_i(vel_estimate, *popt))
plt.show()


def get_user_torque(i_measured, velocity, resisting_torque_current, reduction_ratio, torque_constant):
    user_torque = []
    for i, v in zip(i_measured, velocity):
        if v != 0.0:
            dyn_resisting_current = - 0.8 * v / abs(v) * abs(v) ** (1 / 3.2)
            i_user = i - dyn_resisting_current
        else:
            if abs(i) <= resisting_torque_current:
                i_user = 0.0
            else:
                i_user = i
        user_torque.append(torque_constant * i_user / reduction_ratio)
    return user_torque

user_torque = get_user_torque(np.asarray(monitoring_commands["iq_measured"]), np.asarray(monitoring_commands["vel_estimate"]), 0.42, 8 / 36 * 10 / 91, 0.1053225104205947)

window_length = 30
kernel = np.ones(window_length) / window_length
smoothed_user_torque = np.convolve(user_torque, kernel, mode='valid')
plt.plot(np.asarray(monitoring_commands["time"]), user_torque)
plt.plot(np.asarray(monitoring_commands["time"])[int(window_length / 2) - 1: - int(window_length / 2)], smoothed_user_torque)
plt.show()

print(np.mean(user_torque))
# Fit sur setpoint
# [8.27031057e-01 1.30934379e-03 3.24180986e+00]
# measured -0.008663893192693001
# setpoint -0.0030970109746357166

# Fit sur measured
# [0.83492454 -0.00761155  3.21165553]
# measured 0.0002554140361087889
# setpoint 0.005822296254166078

# Fit sur setpoint filtered
# [0.83504094 0.00483589 3.25038644]
# 0.0019410179897133022
# -0.002162853265569786

# Fit sur measured filtered
# [0.80907057 0.0064548  3.47616679]
# 0.00033077764018978615
# -0.0037730936150933026
