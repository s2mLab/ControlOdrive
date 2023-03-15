import json
import numpy as np
import matplotlib.pyplot as plt

with open("power_control_Concentric_35_75.json", "r") as file:
    data = json.load(file)

plt.figure()
plt.plot(data['time'], data['iq_measured'], label="Iq measured")
plt.plot(data['time'], data['iq_setpoint'], label="Iq setpoint")
plt.plot(data['time'], data['i_res'], label="Resisting torque")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.legend()

plt.figure()
plt.title("Velocity")
plt.plot(data['time'], data['velocity'], label="Estimated velocity")
# plt.plot(data['time'], data['instruction'], label="Instruction")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

plt.figure()
plt.title("Torque")
plt.plot(data['time'], data['instruction'], label="Instruction")
plt.plot(data['time'], data["user_torque"], label="User torque")
plt.plot(data['time'], data["measured_torque"], label="Measured torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.figure()
plt.title("Mechanical power")
plt.plot(data['time'], data["mechanical_power"], label="Mechanical power")
# plt.plot(data['time'], data["user_power"], label="User power")
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")

plt.show()