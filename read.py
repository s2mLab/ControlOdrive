import numpy as np
import matplotlib.pyplot as plt
from save_and_load import read

from enums import (
    ControlMode,
    ODriveMotorError,
    ODriveSensorlessEstimatorError,
    ODriveError,
    ODriveAxisError,
    ODriveEncoderError,
    ODriveControllerError,
)

from utils import traduce_error

data = read("XP/XP_torque.bio", 100, 100)

# Comments
for time, comment in zip(data["time"], data['comments']):
    if comment != "":
        print(f"{time}: {comment}")

# Cadence
instruction_for_cadence = np.zeros(len(data["instruction"]))
instruction_for_cadence[:] = np.nan
instruction_for_cadence[data["control_mode"] == ControlMode.CADENCE_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.CADENCE_CONTROL.value]
instruction_for_cadence[data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value]

plt.figure()
plt.title("Cadence")
plt.ylabel("Cadence (rpm)")
plt.xlabel("Time (s)")

plt.plot(data["time"], data["cadence"], label="Cadence")
plt.plot(
    data["time_for_smoothed"],
    data["smoothed_cadence"],
    label="Smoothed cadence"
)
plt.plot(data["time"], instruction_for_cadence, label="Instruction")

plt.legend()

# Torques
instruction_for_torque = np.zeros(len(data["instruction"]))
instruction_for_torque[:] = np.nan
instruction_for_torque[data["control_mode"] == ControlMode.TORQUE_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.TORQUE_CONTROL.value]
instruction_for_torque[data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value]
instruction_for_torque[data["control_mode"] == ControlMode.LINEAR_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.LINEAR_CONTROL.value]

spinbox_for_torque = np.zeros(len(data["spin_box"]))
spinbox_for_torque[:] = np.nan
spinbox_for_torque[data["control_mode"] == ControlMode.TORQUE_CONTROL.value] = \
    data["spin_box"][data["control_mode"] == ControlMode.TORQUE_CONTROL.value]

fig, ax1 = plt.subplots()

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Torque (Nm)')
ax2.set_ylabel('Cadence (rpm)')

plt.title("Torques")

ax2.plot(
    data["time_for_smoothed"], data['smoothed_cadence'],
    label="Smoothed cadence",
    color='k'
)
ax1.plot(
    data["time_for_smoothed"],
    data["smoothed_motor_torque"],
    label="Smoothed motor torque"
)
ax1.plot(
    data["time_for_smoothed"],
    data["smoothed_user_torque"],
    label="Smoothed user torque"
)
ax1.plot(
    data["time_for_smoothed"],
    data["smoothed_resisting_torque"],
    label="Smoothed resisting torque"
)
ax1.plot(
    data["time"],
    spinbox_for_torque,
    label="Spin box"
)
ax1.plot(
    data["time"],
    instruction_for_torque,
    label="Instruction",
)

fig.legend(loc="upper right")

# Powers
spinbox_for_power = np.zeros(len(data["instruction"]))
spinbox_for_power[:] = np.nan
spinbox_for_power[data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value]
spinbox_for_power[data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value] = \
    data["instruction"][data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value]

fig, ax1 = plt.subplots()
plt.title("Powers")

ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Powers(W)')
ax2.set_ylabel('Cadence (rpm)')

ax1.plot(
    data["time_for_smoothed"],
    data["smoothed_user_power"],
    label="User power"
)
ax1.plot(
    data["time"],
    spinbox_for_power,
    label="Spin_box"
)
ax2.plot(
    data["time_for_smoothed"],
    data["smoothed_cadence"],
    label="Smoothed cadence",
    color='k')

fig.legend(loc="upper right")

# Errors
# plt.figure()
#
# plt.title("Errors")
#
# plt.plot(data["error"], label="Error")
# plt.plot(data["axis_error"], label="Axis error")
# plt.plot(data["controller_error"], label="Controller error")
# plt.plot(data["encoder_error"], label="Encoder error")
# plt.plot(data["motor_error"], label="Motor error")
# plt.plot(data["sensorless_estimator_error"], label="Sensorless estimator error")
# plt.plot(data["can_error"], label="Can error")

print(
    f"{traduce_error(data['error'][-1], ODriveError)}"
    f"{traduce_error(data['axis_error'][-1], ODriveAxisError)}"
    f"{traduce_error(data['controller_error'][-1], ODriveControllerError)}"
    f"{traduce_error(data['encoder_error'][-1], ODriveEncoderError)}"
    f"{traduce_error(data['motor_error'][-1], ODriveMotorError)}"
    f"{traduce_error(data['sensorless_estimator_error'][-1], ODriveSensorlessEstimatorError)}")

plt.show()
