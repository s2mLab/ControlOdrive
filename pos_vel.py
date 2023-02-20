"""Test of changing control modes"""

import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
# motor.calibration()

# Variables to plot
t = []
x_instructions = []
v_instructions = []
pos_estimate = []
vel_estimate = []

t0 = time.time()
t1 = time.time()
t_next = 0

v_instruction = 2.0
x_instruction = np.nan
motor.velocity_control(v_instruction)

while t1 - t0 < 5.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        x_instructions.append(x_instruction)
        pos_estimate.append(np.nan)
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

v_instruction = np.nan
x_instruction = 0.0
motor.position_control(x_instruction)

while t1 - t0 < 15.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        x_instructions.append(x_instruction)
        pos_estimate.append(motor.get_angle_motor())
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

v_instruction = np.nan
x_instruction = 270.0
motor.position_control(x_instruction)

while t1 - t0 < 25.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        x_instructions.append(x_instruction)
        pos_estimate.append(motor.get_angle_motor())
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

v_instruction = -2.0
x_instruction = np.nan
motor.velocity_control(v_instruction)

while t1 - t0 < 30.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        x_instructions.append(x_instruction)
        pos_estimate.append(np.nan)
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

motor.stop()

pos_estimate = np.asarray(pos_estimate)
vel_estimate = np.asarray(vel_estimate)

fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(t, pos_estimate * 360, label="Position")
axs[0].plot(t, x_instructions, label="Instruction")
axs[0].set_title("Position")
axs[0].set_ylabel("Angle (deg)")

axs[1].plot(t, vel_estimate, label="Velocity")
axs[1].plot(t, v_instructions, label="Instruction")
axs[1].set_title("Velocity")
axs[1].set_ylabel("Velocity (tr/s)")
axs[1].legend()
axs[1].set_xlabel("Time (s)")

plt.show()
