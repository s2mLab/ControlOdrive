import json
import numpy as np
import matplotlib.pyplot as plt

with open("120_ecc.json", "r") as file:
    data = json.load(file)

vel_estimate = np.asarray(data["vel_estimate"])
iq_setpoint = np.asarray(data["iq_setpoint"])
iq_measured = np.asarray(data["iq_measured"])
torque = np.asarray(data["torque"])
mechanical_power = np.asarray(data["mechanical_power"])

plt.plot(iq_measured)
plt.plot(iq_setpoint)

plt.figure()
plt.plot(torque)

plt.figure()
plt.plot(vel_estimate)

plt.figure()
plt.plot(mechanical_power)
plt.plot(-torque * vel_estimate * 2 * np.pi / 60)

plt.show()