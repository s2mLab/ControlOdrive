""" Example on how to use the Motor.py code in torque control"""
import json
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
motor.zero_position_calibration()

# Set the control mode
motor.torque_control()

# Variables to plot
t = []
instructions = []
vel_estimate = []
i_m = []
user_torque = []
positions = []

t0 = time.time()
t1 = time.time()
t_next = 0

l = 0.17
m = 4.430
g = 9.81
instruction = 2
print(instruction)
torque_ramp_rate = 5
mode = "Concentric"
motor.set_training_mode(mode)

motor.torque_control(instruction, torque_ramp_rate/10)

while t1 - t0 < 10:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        instructions.append(instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        user_torque.append(motor.get_user_torque())
        i_m.append(motor.get_iq_measured())
        positions.append(motor.get_angle())

        t_next += 0.05

motor.stop()

dictionary = {
    "time": t,
    "vel_estimate": vel_estimate,
    "Iq_measured": i_m,
}

# Writing to .json
json_object = json.dumps(dictionary, indent=4)
with open(f"find_resisting_torque_{mode}_{instruction}_{torque_ramp_rate}_5.json", "w") as outfile:
    outfile.write(json_object)

vel_estimate = np.asarray(vel_estimate)
positions = np.asarray(positions)

plt.title("Velocity in torque control")
plt.plot(t, vel_estimate, label="Estimated velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()

plt.figure()
plt.title("Torque")
plt.plot(t, instructions, label="Instruction")
plt.plot(t, i_m, label="Measured torque")
plt.plot(t, user_torque, label="User torque")
# plt.plot(t, [- l * m * g] * len(t), label="Torque at 90deg")
plt.plot(t, - l * m * g * np.sin(positions / 180 * np.pi), label="'Actual' user torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.show()