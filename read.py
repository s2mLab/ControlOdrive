import numpy as np
import matplotlib.pyplot as plt

from save_and_load import load

from enums import(
    ODriveMotorError,
    ODriveSensorlessEstimatorError,
    ODriveError,
    ODriveAxisError,
    ODriveEncoderError,
    ODriveControllerError,
)

data = load("XP/Kevin_torque_ecc_crash_1.bio")
#577

# t_pb = data['time'][np.where(np.asarray(data["error"]) != 0)[0][0]]

iq = np.asarray(data['iq_measured'])
i_pb = np.where(iq == iq[-1])[0][0]
# Définir la longueur de la fenêtre de moyenne glissante
window_length = 20

# Définir le noyau de moyenne glissante /!\ introduit un biais car intervalles de temps pas constants
kernel = np.ones(window_length) / window_length
smoothed_velocity = np.convolve(data['velocity'], kernel, mode='valid')
smoothed_i_measured = np.convolve(data['iq_measured'], kernel, mode='valid')
smoothed_i_setpoint = np.convolve(data['iq_setpoint'], kernel, mode='valid')
smoothed_measured_torque = np.convolve(data['measured_torque'], kernel, mode='valid')
smoothed_user_torque = np.convolve(data['user_torque'], kernel, mode='valid')
smoothed_user_power = np.convolve(data['user_power'], kernel, mode='valid')
smoothed_mechanical_power = np.convolve(data['mechanical_power'], kernel, mode='valid')
smoothed_electrical_power = np.convolve(data['electrical_power'], np.ones(30) / 30, mode='valid')

print(
    data['velocity'][i_pb],
    data['iq_measured'][i_pb],
    data['iq_setpoint'][i_pb],
    data['electrical_power'][i_pb],
    data['vbus'][i_pb],
    )

# Currents
plt.figure()

plt.title("Currents")

plt.plot(data['time'], np.asarray(data['iq_setpoint']), label="Iq setpoint")
plt.plot(data['time'][9: -10], smoothed_i_measured, label="Smoothed Iq measured")
plt.plot(data['time'][9: -10], smoothed_i_setpoint, label="Smoothed Iq setpoint")
print(min(data['iq_measured']), data['time'][np.where(np.asarray(data['iq_measured']) == min(data['iq_measured']))[0][0]])
plt.legend(loc="upper right")

# Currents
fig, ax1 = plt.subplots()

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Current (A)')
ax2.set_ylabel('Velocity (tr/min)')

ax2.plot(data['time'], - np.asarray(data["velocity"]), label="- Smoothed velocity", color='tab:red')
# ax2.plot([t_pb, t_pb], [min(- smoothed_velocity), max(- smoothed_velocity)])

plt.title("Currents and velocity")

ax1.plot(data['time'], data["iq_measured"], label="Smoothed Iq measured")
# ax1.plot(data['time'], data["motor_error"])
ax1.plot(data['time'], data["iq_setpoint"], label="Smoothed Iq setpoint")
# ax1.plot(data['time'], data["instruction"], label="Instruction")
# ax1.plot(data['time'], data["spin_box"], label="Spin box")
ax1.plot(data['time'], data["resistor_current"], label="Brake resistor")
fig.legend(loc="upper right")

# # Velocity
# plt.figure()
# plt.title("Velocity")
# plt.plot(data['time'], data['velocity'], label="Velocity")
# plt.plot(data['time'], data['instruction'], label="Instruction")
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (tr/min)")
# plt.legend()

# Torques
fig, ax1 = plt.subplots()

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Torque (Nm)')
ax2.set_ylabel('Velocity (tr/min)')

plt.title("Torques")

ax2.plot(data['time'][9: -10], smoothed_velocity, label="Smoothed velocity", color='tab:red')
ax1.plot(data['time'][9: -10], smoothed_measured_torque, label="Smoothed measured torque")
ax1.plot(data['time'][9: -10], - smoothed_user_torque, label="Smoothed user torque")
ax2.plot(data['time'], data["instruction"], label="Instruction")
fig.legend(loc="upper right")

# plt.figure()
# plt.title("Torque")
# plt.plot(data['time'], data["user_torque"], label="User torque")
# plt.plot(data['time'], np.max(np.asarray([list(- 2 * np.asarray(data['time'])), list(np.asarray(data['instruction']))]), axis=0), label="Instruction")
# plt.xlabel("Time (s)")
# plt.ylabel("Torque (Nm)")
# plt.legend()

# Powers
fig, ax1 = plt.subplots()
plt.title("Powers")

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Powers(W)')
ax2.set_ylabel('Velocity (tr/min)')

ax1.plot(data['time'][14: -15], smoothed_electrical_power, label="Smoothed electrical power")
ax1.plot(data['time'][9: -10], smoothed_mechanical_power, label="Smoothed mechanical power (user + resisting torque)")
ax1.plot(data['time'][9: -10], abs(smoothed_user_power), label="User power")
ax2.plot(data['time'][9: -10], smoothed_velocity, label="Smoothed velocity", color='tab:red')

fig.legend(loc="upper right")

# vbus
fig, ax1 = plt.subplots()

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('vbus')
ax2.set_ylabel('Velocity (tr/min)')

ax1.plot(data['time'], data["vbus"], label="vbus")
ax2.plot(data['time'][9: -10], smoothed_velocity, label="Smoothed velocity", color='tab:red')

plt.title("vbus and velocity")
fig.legend(loc="upper right")

# ibus
fig, ax1 = plt.subplots()

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Current (A)')
ax2.set_ylabel('Velocity (tr/min)')

dibusdt = (np.asarray(data["ibus"])[1:] - np.asarray(data["ibus"])[:-1])/(np.asarray(data["time"])[1:] - np.asarray(data["time"])[:-1])
ax1.plot(data['time'], data["ibus"], label="ibus")
ax1.plot(data['time'], data["resistor_current"], label="Resistor current")
ax2.plot(data['time'][9: -10], smoothed_velocity, label="Smoothed velocity", color='tab:red')
ax2.plot(data['time'], data['instruction'], label="Instruction", color='tab:blue')
# ax2.plot(data['time'], data['spin_box'], label="Spin box", color='tab:green')
fig.legend(loc="upper right")

# Issue 16
fig, ax1 = plt.subplots()

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Saturated (True/False)')
ax2.set_ylabel('Velocity (tr/min)')

ax1.plot(data["brake_resistor_saturated"], label="Saturated")
ax2.plot(data["velocity"], label="Velocity", color='tab:red')

print(min(data["ibus"]))
plt.title("brake_resistor_saturated and velocity")
fig.legend(loc="upper right")

# Errors
plt.figure()
plt.title("Errors")
plt.plot(data["error"])
plt.plot(data["axis_error"])
plt.plot(data["controller_error"])
plt.plot(data["encoder_error"])
plt.plot(data["motor_error"])
plt.plot(data["sensorless_estimator_error"])


def _traduce_error(decim_number, odrive_enum):
    hex_number = "{0:x}".format(decim_number)
    res = ""
    for i, j in enumerate(hex_number[::-1]):
        if j != '0':
            for member in odrive_enum.__members__.values():
                if member.value == "0x" + format(int(j) * 16 ** int(i), f"0{8}X"):
                    res += member.name
                    res += ", "
    return res

print(
    f"{_traduce_error(data['error'][-1], ODriveError)}"
    f"{_traduce_error(data['axis_error'][-1], ODriveAxisError)}"
    f"{_traduce_error(data['controller_error'][-1], ODriveControllerError)}"
    f"{_traduce_error(data['encoder_error'][-1], ODriveEncoderError)}"
    f"{_traduce_error(data['motor_error'][-1], ODriveMotorError)}"
    f"{_traduce_error(data['sensorless_estimator_error'][-1], ODriveSensorlessEstimatorError)}")

plt.legend()

plt.show()