"""
"""
import matplotlib.pyplot as plt
import numpy as np
from ergocycleS2M.data_processing.load import load

data = load("/home/mickaelbegon/Documents/Stage_Amandine/ControlOdrive/XP/Calib_torque.bio")

time = np.asarray(data["time"])
iq_measured = np.asarray(data["iq_measured"])
vel_estimate = np.asarray(data["vel_estimate"])

plt.plot(time[vel_estimate == 0], iq_measured[vel_estimate == 0])
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.show()