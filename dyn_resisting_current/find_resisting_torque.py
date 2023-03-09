""" Example on how to use the Motor.py code in velocity control"""
import json
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
motor.zero_position_calibration()

# Set the control mode
motor.velocity_control(-0.7)
time.sleep(10)

# Variables to plot
t = []
instructions = []
angle = []
vel_estimate = []
iq_setpoint = []
iq_measured = []
electrical_power = []
i_res = []

t0 = time.time()
t1 = time.time()
t_next = 0

fs = 50

# for instruction, delta_t, mode in zip((0.25, 0.5, -0.25), (15, 30, 45), ("Eccentric", "Eccentric", "Concentric")):
for i in range(-14, 15):
    instruction = 0.05 * i
    if i > 0:
        motor.set_training_mode("Eccentric")
    motor.velocity_control(instruction)
    print(instruction)

    while t1 - t0 < 20 * i + (14 * 20 + 20):
        t1 = time.time()
        if t1 - t0 > t_next:
            t.append(t1 - t0)
            instructions.append(instruction)
            angle.append(motor.get_angle())
            vel_estimate.append(motor.get_estimated_velocity())
            iq_setpoint.append(motor.get_iq_setpoint())
            iq_measured.append(motor.get_iq_measured())
            electrical_power.append(motor.get_electrical_power())
            i_res.append(motor.get_i_res())
            t_next += 1/fs

motor.stop()

dictionary = {
    "time": t,
    "angle": angle,
    "vel_estimate": vel_estimate,
    "iq_setpoint": iq_setpoint,
    "iq_measured": iq_measured,
    "electrical_power": electrical_power
}

t = np.asarray(t)
angle = np.asarray(angle)
vel_estimate = np.asarray(vel_estimate)
iq_setpoint = np.asarray(iq_setpoint)
iq_measured = np.asarray(iq_measured)
electrical_power = np.asarray(electrical_power)

# Writing to .json
json_object = json.dumps(dictionary, indent=4)
with open("find_resisting_torque4.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(t, vel_estimate, label="Estimated velocity")
plt.plot(t, instructions, label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (turn/s)")
plt.legend()

fig2, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(t, electrical_power, label="Electrical power")
axs[0].set_title("Electrical power")
axs[1].plot(t, iq_measured, label="Iq_measured")
axs[1].plot(t, iq_setpoint, label="Iq_setpoint")
axs[1].set_title("Current")
axs[1].legend()
axs[2].plot(t, angle)
axs[2].set_title("Angle")

plt.figure()
plt.plot(t, iq_measured, label="Current measured")
plt.plot(t, iq_setpoint, label="Current setpoint")
plt.plot(t, i_res, label="Current corresponding to the resisting torque")
plt.plot(t, i_res, label="Current measured corresponding to the user produced torque")
plt.title("Current")
plt.legend()

plt.show()