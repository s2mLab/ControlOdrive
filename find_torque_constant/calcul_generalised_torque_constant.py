import json
import numpy as np
import matplotlib.pyplot as plt

l = 0.17
g = 9.81
r = 8/36*10/91
# weight_torque_at_motor is the torque produced by the weight + the pedals multiplied by r
weight_torque_at_motor = []
# the setpoint as been chosen instead of the measures because there is a lot of noise on the measures
i_setpoint = []
resisting_current = 0.41654785903687247

for side in ("left", "right"):
    for weight, m in zip(("3kg", "10p"), (3, 4.53592)):
        with open(f"find_torque_constant/torque_constant_{side}_{weight}.json", "r") as file:
            data = json.load(file)

        time = np.asarray(data["time"])
        iq_setpoint = np.asarray(data["iq_setpoint"])
        positions = np.asarray(data["positions"])

        iq_setpoint[abs(iq_setpoint) < resisting_current] = 0.0
        iq_setpoint[iq_setpoint <= - resisting_current] += resisting_current
        iq_setpoint[iq_setpoint >= resisting_current] -= resisting_current

        positions[positions > 337.0] -= 360.0  # for the means to be accurate

        # the seconds at the begining and the end of each measured that are not taken into the account because of the
        # movement
        dt = 2.0

        for i in range(8):
            # only taking the central data for each angle
            T = time[time > i * 12 + dt]
            I = iq_setpoint[time > i * 12 + dt]
            P = positions[time > i * 12 + dt]
            I = I[T < (i + 1) * 12 - dt]
            P = P[T < (i + 1) * 12 - dt]

            i_setpoint.append(np.mean(I))
            weight_torque_at_motor.append(- m * l * g * np.sin(np.mean(P)/180*np.pi) * r)

torque_constant, b = np.polyfit(i_setpoint, weight_torque_at_motor, 1)
print(torque_constant, b)

# Before subtracting resisting current: 0.08225583064524448 0.0023268897548210173
# After subtracting resisting current: 0.10680746759421 0.0054162519079533865

plt.plot(weight_torque_at_motor, label="T")
plt.plot(torque_constant * np.asarray(i_setpoint), label="Tcalc")
plt.title("Position control")
plt.xlabel("Time (s)")
plt.ylabel("Position (deg)")
plt.legend()

plt.show()
