import numpy as np
import matplotlib.pyplot as plt

from save_and_load import load

data = load("velocity_control_Eccentric_30_392.bio")

iq = np.asarray(data['iq_measured'])
i_pb = np.where(iq == iq[-1])[0][0]

print(
    data['velocity'][i_pb],
    data['iq_measured'][i_pb],
    data['iq_setpoint'][i_pb],
    data['electrical_power'][i_pb],
    data['vbus'][i_pb],
    )

plt.figure()
plt.plot(data['time'], data['iq_measured'], label="Iq measured")
plt.plot(data['time'], data['iq_setpoint'], label="Iq setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.legend()

plt.figure()
plt.title("Velocity")
plt.plot(data['time'], data['velocity'], label="Estimated velocity")
# plt.plot(data['time'], 10 * np.sin(np.asarray(data['angle']) / 180 * np.pi) + 60, label="Angle")
plt.plot(data['time'], data['instruction'], label="Instruction")
# plt.plot(data['time'][i_pb], data['velocity'][i_pb], "+", label="Estimated velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

plt.figure()
plt.title("Torque")
plt.plot(data['time'], data["user_torque"], label="User torque")
# plt.plot(data['time'], data["measured_torque"], label="Measured torque")
plt.plot(data['time'], np.max(np.asarray([list(- 2 * np.asarray(data['time'])), list(np.asarray(data['instruction']))]), axis=0), label="Instruction")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.figure()
plt.title("Mechanical power")
plt.plot(data['time'], data["mechanical_power"], label="Mechanical power (user + resisting torque)")
plt.plot(data['time'], - np.asarray(data["user_power"]) * 2 * np.pi / 60, label="User power")
plt.plot(data['time'], [-35] * len(data['time']), label="Power instruction")
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")
plt.legend()

plt.figure()
plt.title("Electrical power")
plt.plot(data['time'], data["electrical_power"], label="Electrical power")
plt.plot(data['time'][i_pb], data["electrical_power"][i_pb], "+", label="Electrical power")
plt.legend()

plt.figure()
plt.title("vbus")
plt.plot(data['time'], data["vbus"])
plt.plot(data['time'][i_pb], data["vbus"][i_pb], "+")
plt.legend()

plt.show()