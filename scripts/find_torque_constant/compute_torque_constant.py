"""
This script was used to compute the torque constant of the Ergocycle.
"""
import json
import numpy as np
import matplotlib.pyplot as plt

length = 0.17
g = 9.81
r = 8 / 36 * 10 / 91

# weight_torque_at_motor is the torque produced by the weight + the pedals multiplied by r
weight_torque_at_motor = []

# The setpoint has been chosen instead of the measures because there is a lot of noise on the measures
iq_util = []

resisting_current = 0.42126813627221144  # Calculated with `find_resisting_torque/compute resisting_torque`

for side in ("left", "right"):
    for weight, m in zip(("3kg", "10p"), (3, 4.430)):
        with open(f"find_torque_constant/torque_constant_{side}_{weight}.json", "r") as file:
            data = json.load(file)

        time = np.asarray(data["time"])
        iq_setpoint = np.asarray(data["iq_setpoint"])
        positions = np.asarray(data["positions"])

        # Subtraction of the current corresponding to the resisting torque -> the remaining current is the util current
        iq_setpoint[abs(iq_setpoint) < resisting_current] = 0.0
        iq_setpoint[iq_setpoint <= -resisting_current] += resisting_current
        iq_setpoint[iq_setpoint >= resisting_current] -= resisting_current

        positions[positions > 337.0] -= 360.0  # for the means to be accurate

        # The seconds at the beginning and the end of each measured that are not taken into the account because of the
        # movement
        dt = 2.0

        for i in range(8):
            # only taking the central data for each angle
            T = time[time > i * 12 + dt]
            C = iq_setpoint[time > i * 12 + dt]
            P = positions[time > i * 12 + dt]
            C = C[T < (i + 1) * 12 - dt]
            P = P[T < (i + 1) * 12 - dt]

            iq_util.append(np.mean(C))
            weight_torque_at_motor.append(-m * length * g * np.sin(np.mean(P) / 180 * np.pi) * r)

torque_constant, b = np.polyfit(iq_util, weight_torque_at_motor, 1)
print(torque_constant, b)

# Before subtracting resisting current: 0.08092403821181135 0.0022894644134235005
# After subtracting resisting current: 0.1053225104205947 0.005346468840094937

plt.plot(weight_torque_at_motor, label="T")
plt.plot(torque_constant * np.asarray(iq_util), label="Tcalc")
plt.title("Position control")
plt.xlabel("Time (s)")
plt.ylabel("Position (deg)")
plt.legend()

plt.show()
