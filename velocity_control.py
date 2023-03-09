""" Example on how to use the Motor.py code in velocity control"""
import json
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
# motor.calibration()

# Set the control mode
motor.velocity_control(0)

# Variables to plot
t = []
instructions = []
vel_estimate = []
iq_setpoint = []
iq_measured = []
electrical_power = []
torque = []
vbus = []

t0 = time.time()
t1 = time.time()
t_next = 0

#for instruction, delta_t, mode in zip((15, 30, -15), (15, 30, 45), ("Eccentric", "Eccentric", "Concentric")):
#for i in range(1, 6):
instruction = -30
#motor.set_training_mode("Eccentric")
motor.velocity_control(instruction)
print(instruction)

while t1 - t0 < 20:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        instructions.append(instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        iq_setpoint.append(motor.get_iq_setpoint())
        iq_measured.append(motor.get_iq_measured())
        electrical_power.append(motor.get_electrical_power())
        vbus.append(motor.get_monitoring_commands()[2])
        torque.append(motor.get_torque_measured())
        t_next += 0.05

motor.stop()

dictionary = {
    "vel_estimate": vel_estimate,
    "iq_setpoint": iq_setpoint,
    "iq_measured": iq_measured,
    "electrical_power": electrical_power
}

vel_estimate = np.asarray(vel_estimate)
iq_setpoint = np.asarray(iq_setpoint)
iq_measured = np.asarray(iq_measured)
electrical_power = np.asarray(electrical_power)

# Writing to .json

json_object = json.dumps(dictionary, indent=4)
with open("XP/monitoring_commands.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(t, vel_estimate, label="Estimated velocity")
plt.plot(t, instructions, label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

fig2, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(electrical_power, label="Electrical power")
axs[0].set_title("Electrical power")
axs[1].plot(iq_setpoint, label="Iq_setpoint")
axs[1].plot(iq_measured, label="Iq_measured")
axs[1].set_title("Current")
axs[1].legend()

plt.show()