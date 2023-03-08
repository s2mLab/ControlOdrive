"""Test of changing control modes"""

import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
# motor.calibration()

# Variables to plot
t = []
v_instructions = []
vel_estimate = []

t0 = time.time()
t1 = time.time()
t_next = 0

v_instruction = -0.5
motor.velocity_control(v_instruction)

while t1 - t0 < 5.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

v_instruction = np.nan
motor.position_control(2)

while t1 - t0 < 15.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

v_instruction = np.nan
motor.set_training_mode("Eccentric")
motor.position_control(-3)

while t1 - t0 < 25.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

v_instruction = 0.5
motor.velocity_control(v_instruction)

while t1 - t0 < 30.0:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        v_instructions.append(v_instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        t_next += 0.05

motor.stop()

vel_estimate = np.asarray(vel_estimate)

plt.plot(t, vel_estimate, label="Velocity")
plt.plot(t, v_instructions, label="Instruction")
plt.ylabel("Velocity (tr/s)")
plt.legend()
plt.xlabel("Time (s)")

plt.show()