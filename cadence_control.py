"""
Example on how to use the motor.py code in cadence control
"""
import matplotlib.pyplot as plt
import numpy as np
import time

from ergocycleS2M.motor_control.motor import MotorController
from ergocycleS2M.data_processing.load import load, compute_data

# Initialisation
motor = MotorController()

# Data saving
t0 = time.time()
t1 = time.time()
f_sample = 20  # Hz
t_next = 0

# Control
instruction = 30
ramp_rate = 6
motor.set_direction("Reverse")
motor.cadence_control(instruction, cadence_ramp_rate=ramp_rate)

while t1 - t0 < 10:
    t1 = time.time()
    if t1 - t0 > t_next:
        motor.minimal_save_data_to_file(
            "cadence_control_example", spin_box=instruction, instruction=instruction, ramp_instruction=ramp_rate
        )
        t_next += 1 / f_sample

motor.stop()

data = load("cadence_control_example.bio")

t = np.asarray(data["time"])

# Plot
_, cadence, *_ = compute_data(data["vel_estimate"], data["turns"], data["iq_measured"])
plt.plot(t, cadence, label="Pedals cadence")
plt.plot(t, - np.min([list(ramp_rate * t), data["instruction"]], axis=0), label="Instruction")
plt.title("Ramped cadence control")
plt.xlabel("Time (s)")
plt.ylabel("Cadence (rpm)")
plt.legend()

plt.show()