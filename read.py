import numpy as np
import matplotlib.pyplot as plt
from save_and_load import load

from enums import (
    ODriveMotorError,
    ODriveSensorlessEstimatorError,
    ODriveError,
    ODriveAxisError,
    ODriveEncoderError,
    ODriveControllerError,
)

data = load("XP/XP_participantX(4).bio")

time = np.asarray(data["time"])
dt = time[1:] - time[:-1]
print(dt.mean(), dt.std())

# Moving average
window_length = 20
# /!\ bias because the time steps are not the same
kernel = np.ones(window_length) / window_length
smoothed_cadence = np.convolve(data['cadence'], kernel, mode='valid')
smoothed_i_measured = np.convolve(data['iq_measured'], kernel, mode='valid')
smoothed_i_setpoint = np.convolve(data['iq_setpoint'], kernel, mode='valid')
smoothed_motor_torque = np.convolve(data['motor_torque'], kernel, mode='valid')
smoothed_user_torque = np.convolve(data['user_torque'], kernel, mode='valid')
smoothed_user_power = np.convolve(data['user_power'], kernel, mode='valid')
smoothed_mechanical_power = np.convolve(data['mechanical_power'], kernel, mode='valid')
smoothed_electrical_power = np.convolve(data['electrical_power'], kernel, mode='valid')

for time, comment in zip(data["time"], data['comments']):
    if comment != "":
        print(f"{time}: {comment}")

# Stopwatch
plt.figure()
plt.title("Stopwatch")
plt.ylabel("Time (s)")
plt.xlabel("Time (s)")

plt.plot(data['time'], data['stopwatch'], label="Stopwatch")
plt.plot(data['time'], data['lap'], label="Lap")

plt.legend()

# cadence
plt.figure()
plt.title("cadence")
plt.ylabel("cadence (rpm)")
plt.xlabel("Time (s)")

plt.plot(data['time'], data['cadence'], label="cadence")
plt.plot(
    data['time'][int(window_length / 2) - 1: int(- window_length / 2)],
    smoothed_cadence,
    label="Smoothed cadence"
)
plt.plot(data['time'], np.asarray(data['instruction']), label="Instruction")

plt.legend()

# Instructions
plt.figure()
plt.title("Instructions")
plt.ylabel("Instruction")
plt.xlabel("Time (s)")

plt.plot(data['time'], data['ramp_instruction'], label="Ramp instruction")
plt.plot(data['time'], data['instruction'], label="Instruction")
plt.plot(data['time'], data['spin_box'], label="Spin box")

plt.legend()

# Currents
plt.figure()

plt.title("Currents")

plt.ylabel("Current (A)")
plt.xlabel("Time (s)")

plt.plot(data['time'], np.asarray(data['iq_measured']), label="Iq measured")
plt.plot(data['time'], np.asarray(data['iq_setpoint']), label="Iq setpoint")

plt.legend(loc="upper right")

# Currents
fig, ax1 = plt.subplots()

plt.title("Currents and cadence")

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Current (A)')
ax2.set_ylabel('cadence (rpm)')

ax2.plot(data['time'][int(window_length / 2) - 1: int(- window_length / 2)], smoothed_cadence, label="Smoothed cadence", color='k')
ax1.plot(data['time'], data["iq_measured"], label="Iq measured")
ax1.plot(data['time'], data["iq_setpoint"], label="Iq setpoint")
ax1.plot(data['time'], data["resistor_current"], label="Brake resistor")

fig.legend(loc="upper right")

# Torques
fig, ax1 = plt.subplots()

# ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Torque (Nm)')
# ax2.set_ylabel('cadence (rpm)')

plt.title("Torques")

# ax2.plot(
#     data['time'][int(window_length / 2) - 1: int(- window_length / 2)], smoothed_cadence,
#     label="Smoothed cadence",
#     color='k'
# )
ax1.plot(
    data['time'][int(window_length / 2) - 1: int(- window_length / 2)],
    smoothed_motor_torque,
    label="Smoothed motor torque"
)
ax1.plot(
    data['time'][int(window_length / 2) - 1: int(- window_length / 2)],
    smoothed_user_torque,
    label="Smoothed user torque"
)
ax1.plot(
    data['time'],
    np.asarray(data['instruction']),
    label="Instruction",
)
#ax1.plot(
#    data['time'],
#    np.asarray(data['spin_box']),
#    label="Spin box",
#)
#ax1.plot(
#    data['time'],
#    np.asarray(data['spin_box']) * np.asarray(data['cadence']),
#    label="Spin box * cadence",
#)

fig.legend(loc="upper right")

# Powers
fig, ax1 = plt.subplots()
plt.title("Powers")

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Powers(W)')
ax2.set_ylabel('cadence (rpm)')

ax1.plot(
    data['time'],
    data['electrical_power'],
    label="Electrical power"
)
ax1.plot(
    data['time'],
    data['mechanical_power'],
    label="Mechanical power (user + resisting torque)"
)
ax1.plot(
    data['time'],
    data['user_power'],
    label="User power"
)
ax1.plot(
    data['time'],
    data['spin_box'],
    label="Spin_box"
)
ax2.plot(
    data['time'][int(window_length / 2) - 1: int(- window_length / 2)],
    smoothed_cadence,
    label="Smoothed cadence",
    color='k')

fig.legend(loc="upper right")

# vbus
fig, ax1 = plt.subplots()

plt.title("vbus and cadence of the pedal")

ax2 = ax1.twinx()
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('vbus (V)')
ax2.set_ylabel('cadence (rpm)')

ax1.plot(data['time'], data["vbus"], label="vbus")
ax2.plot(
    data['time'][int(window_length / 2) - 1: int(- window_length / 2)],
    smoothed_cadence, label="Smoothed cadence of the pedals",
    color="k"
)

fig.legend()

# ibus
fig, ax1 = plt.subplots()

fig.suptitle("ibus, resistor current and saturation of the brake resistor")

ax2 = ax1.twinx()
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Current (A)')
ax2.set_ylabel('Brake resistor saturated')

ax1.plot(data['time'], data["ibus"], label="ibus")
ax1.plot(data['time'], data["resistor_current"], label="Resistor current")
ax2.plot(data['time'], data["brake_resistor_saturated"], label="Brake resistor saturated", color="green")

fig.legend(loc="upper right")

# Errors
plt.figure()

plt.title("Errors")

plt.plot(data["error"], label="Error")
plt.plot(data["axis_error"], label="Axis error")
plt.plot(data["controller_error"], label="Controller error")
plt.plot(data["encoder_error"], label="Encoder error")
plt.plot(data["motor_error"], label="Motor error")
plt.plot(data["sensorless_estimator_error"], label="Sensorless estimator error")
plt.plot(data["can_error"], label="Can error")


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